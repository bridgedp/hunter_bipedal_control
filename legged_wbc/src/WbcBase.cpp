//
// Created by qiayuan on 2022/7/1.
//

// some ref: https://github.com/skywoodsz/qm_control

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "legged_wbc/WbcBase.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <utility>

namespace legged
{
WbcBase::WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                 const PinocchioEndEffectorKinematics& eeKinematics)
  : pinocchioInterfaceMeasured_(pinocchioInterface)
  , pinocchioInterfaceDesired_(pinocchioInterface)
  , info_(std::move(info))
  , mapping_(info_)
  , inputLast_(vector_t::Zero(info_.inputDim))
  , eeKinematics_(eeKinematics.clone())
  , rbdConversions_(pinocchioInterface, info_)
{
  // linear force, plus angular force
  contact_force_size_ = 3 * info_.numThreeDofContacts;
  numDecisionVars_ = info_.generalizedCoordinatesNum + contact_force_size_ + info_.actuatedDofNum;  // 16+3*4+10=38
  qMeasured_ = vector_t(info_.generalizedCoordinatesNum);
  vMeasured_ = vector_t(info_.generalizedCoordinatesNum);
  cmd_body_pos_.resize(6);
  cmd_body_pos_.setZero();
  cmd_body_vel_.resize(6);
  cmd_body_vel_.setZero();
  earlyLatecontact_[0].fill(false);
  earlyLatecontact_[1].fill(false);
}

vector_t WbcBase::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured,
                         size_t mode, scalar_t /*period*/)
{
  contactFlag_ = modeNumber2StanceLeg(mode);
  numContacts_ = 0;
  for (bool flag : contactFlag_)
  {
    if (flag)
    {
      numContacts_++;
    }
  }

  updateMeasured(rbdStateMeasured);
  updateDesired(stateDesired, inputDesired);

  return {};
}

void WbcBase::updateMeasured(const vector_t& rbdStateMeasured)
{
  qMeasured_.head<3>() = rbdStateMeasured.segment<3>(3);  // xyz linaer pos
  qMeasured_.segment<3>(3) = rbdStateMeasured.head<3>();
  qMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum);
  vMeasured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3);
  vMeasured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qMeasured_.segment<3>(3), rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum));
  vMeasured_.tail(info_.actuatedDofNum) =
      rbdStateMeasured.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);

  const auto& model = pinocchioInterfaceMeasured_.getModel();
  auto& data = pinocchioInterfaceMeasured_.getData();

  // For floating base EoM task
  pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::crba(model, data, qMeasured_);
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_);
  j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
  {
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
    jac.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
    j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
  }

  pinocchio::computeJointJacobiansTimeVariation(model, data, qMeasured_, vMeasured_);
  dj_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
  {
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
    jac.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobianTimeVariation(model, data, info_.endEffectorFrameIndices[i],
                                             pinocchio::LOCAL_WORLD_ALIGNED, jac);
    dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
  }

  // For base motion tracking task
  base_j_.setZero(6, info_.generalizedCoordinatesNum);
  base_dj_.setZero(6, info_.generalizedCoordinatesNum);
  pinocchio::getFrameJacobian(model, data, model.getBodyId("base_link"), pinocchio::LOCAL_WORLD_ALIGNED, base_j_);
  pinocchio::getFrameJacobianTimeVariation(model, data, model.getBodyId("base_link"), pinocchio::LOCAL_WORLD_ALIGNED,
                                           base_dj_);
}

void WbcBase::updateDesired(const vector_t& stateDesired, const vector_t& inputDesired)
{
  const auto& model = pinocchioInterfaceDesired_.getModel();
  auto& data = pinocchioInterfaceDesired_.getData();

  mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);
  const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);
  pinocchio::forwardKinematics(model, data, qDesired);
  pinocchio::computeJointJacobians(model, data, qDesired);
  pinocchio::updateFramePlacements(model, data);
  updateCentroidalDynamics(pinocchioInterfaceDesired_, info_, qDesired);
  const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);
  pinocchio::forwardKinematics(model, data, qDesired, vDesired);

  const vector_t jointAccelerations = vector_t::Zero(info_.actuatedDofNum);
  rbdConversions_.computeBaseKinematicsFromCentroidalModel(stateDesired, inputDesired, jointAccelerations, basePoseDes_,
                                                           baseVelocityDes_, baseAccelerationDes_);
}

Task WbcBase::formulateFloatingBaseEomTask()
{
  auto& data = pinocchioInterfaceMeasured_.getData();
  matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);
  s.block(0, 0, info_.actuatedDofNum, 6).setZero();
  s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();
  matrix_t a = (matrix_t(info_.generalizedCoordinatesNum, numDecisionVars_) << data.M, -j_.transpose(), -s.transpose())
                   .finished();  
  vector_t b = -data.nle;        

  return { a, b, matrix_t(), vector_t() };
}

Task WbcBase::formulateTorqueLimitsTask()
{
  matrix_t d(2 * info_.actuatedDofNum, numDecisionVars_);
  d.setZero();
  matrix_t i = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);
  d.block(0, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum,
          info_.actuatedDofNum) = i;
  d.block(info_.actuatedDofNum, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum,
          info_.actuatedDofNum) = -i;
  vector_t f(2 * info_.actuatedDofNum);
  const int dofPerLeg = info_.actuatedDofNum / 2;
  for (size_t l = 0; l < 2 * info_.actuatedDofNum / dofPerLeg; ++l)
  {
    f.segment(dofPerLeg * l, dofPerLeg) = torqueLimits_;
  }
  return { matrix_t(), vector_t(), d, f };
}

Task WbcBase::formulateNoContactMotionTask()
{
  matrix_t a(3 * numContacts_, numDecisionVars_);
  vector_t b(a.rows());

  a.setZero();
  b.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; i++)
  {
    if (contactFlag_[i])
    {
      a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
      b.segment(3 * j, 3) = -dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
      j++;
    }
  }

  return { a, b, matrix_t(), vector_t() };
}

Task WbcBase::formulateFrictionConeTask()
{
  matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  a.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
  {
    if (!contactFlag_[i])
    {
      a.block(3 * j++, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
    }
  }
  vector_t b(a.rows());
  b.setZero();

  matrix_t frictionPyramic(5, 3);  // clang-format off
  frictionPyramic << 0, 0, -1,
                     1, 0, -frictionCoeff_,
                    -1, 0, -frictionCoeff_,
                     0, 1, -frictionCoeff_,
                     0,-1, -frictionCoeff_;  // clang-format on

  matrix_t d(5 * numContacts_ + 3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  d.setZero();
  j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
  {
    if (contactFlag_[i])
    {
      d.block(5 * j++, info_.generalizedCoordinatesNum + 3 * i, 5, 3) = frictionPyramic;
    }
  }
  vector_t f = Eigen::VectorXd::Zero(d.rows());

  return { a, b, d, f };
}

// Tracking base xy linear motion task
Task WbcBase::formulateBaseXYLinearAccelTask()
{
  matrix_t a(2, numDecisionVars_);
  vector_t b(a.rows());

  a.setZero();
  b.setZero();

  a.block(0, 0, 2, 2) = matrix_t::Identity(2, 2);
  b = baseAccelerationDes_.segment<2>(0);

  return { a, b, matrix_t(), vector_t() };
}

// Tracking base height motion task
Task WbcBase::formulateBaseHeightMotionTask()
{
  matrix_t a(1, numDecisionVars_);
  vector_t b(a.rows());

  a.setZero();
  b.setZero();
  a.block(0, 2, 1, 1).setIdentity();

  b[0] = baseAccelerationDes_[2] + baseHeightKp_ * (basePoseDes_[2] - qMeasured_[2]) +
         baseHeightKd_ * (baseVelocityDes_[2] - vMeasured_[2]);

  return { a, b, matrix_t(), vector_t() };
}

// Tracking base angular motion task
Task WbcBase::formulateBaseAngularMotionTask()
{
  matrix_t a(3, numDecisionVars_);
  vector_t b(a.rows());

  a.setZero();
  b.setZero();

  a.block(0, 0, 3, info_.generalizedCoordinatesNum) = base_j_.block(3, 0, 3, info_.generalizedCoordinatesNum);

  vector3_t eulerAngles = qMeasured_.segment<3>(3);

  // from derivative euler to angular
  vector3_t vMeasuredGlobal =
      getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(eulerAngles, vMeasured_.segment<3>(3));
  vector3_t vDesiredGlobal = baseVelocityDes_.tail<3>();

  // from euler to rotation
  vector3_t eulerAnglesDesired = basePoseDes_.tail<3>();
  matrix3_t rotationBaseMeasuredToWorld = getRotationMatrixFromZyxEulerAngles<scalar_t>(eulerAngles);
  matrix3_t rotationBaseReferenceToWorld = getRotationMatrixFromZyxEulerAngles<scalar_t>(eulerAnglesDesired);

  vector3_t error = rotationErrorInWorld<scalar_t>(rotationBaseReferenceToWorld, rotationBaseMeasuredToWorld);

  // desired acc
  vector3_t accDesired = baseAccelerationDes_.tail<3>();

  b = accDesired + baseAngularKp_ * error + baseAngularKd_ * (vDesiredGlobal - vMeasuredGlobal) -
      base_dj_.block(3, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;

  return { a, b, matrix_t(), vector_t() };
}

Task WbcBase::formulateBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period)
{
  return formulateBaseXYLinearAccelTask() + formulateBaseHeightMotionTask() + formulateBaseAngularMotionTask();
}

Task WbcBase::formulateSwingLegTask()
{
  eeKinematics_->setPinocchioInterface(pinocchioInterfaceMeasured_);
  std::vector<vector3_t> posMeasured = eeKinematics_->getPosition(vector_t());
  std::vector<vector3_t> velMeasured = eeKinematics_->getVelocity(vector_t(), vector_t());
  eeKinematics_->setPinocchioInterface(pinocchioInterfaceDesired_);
  std::vector<vector3_t> posDesired = eeKinematics_->getPosition(vector_t());
  std::vector<vector3_t> velDesired = eeKinematics_->getVelocity(vector_t(), vector_t());

  matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();
  b.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
  {
    if (!contactFlag_[i])
    {
      vector3_t accel = swingKp_ * (posDesired[i] - posMeasured[i]) + swingKd_ * (velDesired[i] - velMeasured[i]);
      a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
      b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
      j++;
    }
  }

  return { a, b, matrix_t(), vector_t() };
}

Task WbcBase::formulateContactForceTask(const vector_t& inputDesired) const
{
  matrix_t a(3 * info_.numThreeDofContacts, numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();

  for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
  {
    a.block(3 * i, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
  }
  b = inputDesired.head(a.rows());

  return { a, b, matrix_t(), vector_t() };
}

void WbcBase::compensateFriction(vector_t& x)
{
  vector_t coulomb_friction(info_.actuatedDofNum);
  vector_t joint_v = vMeasured_.tail(info_.actuatedDofNum);
  for (int i = 0; i < info_.actuatedDofNum; i++)
  {
    const int sgn = (joint_v[i] > 0) - (joint_v[i] < 0);
    coulomb_friction[i] = (abs(joint_v[i]) > 0.001) ? (sgn * 0.2) : 0;
  }
  x.tail(12) = x.tail(12) + coulomb_friction;
}

void WbcBase::loadTasksSetting(const std::string& taskFile, bool verbose)
{
  // Load task file
  torqueLimits_ = vector_t(info_.actuatedDofNum / 2);
  loadData::loadEigenMatrix(taskFile, "torqueLimitsTask", torqueLimits_);
  if (verbose)
  {
    std::cerr << "\n #### Torque Limits Task:";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "\n #### motor1, motor2, motor3, motor4, motor5: " << torqueLimits_.transpose() << "\n";
    std::cerr << " #### =============================================================================\n";
  }
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "frictionConeTask.";
  if (verbose)
  {
    std::cerr << "\n #### Friction Cone Task:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, frictionCoeff_, prefix + "frictionCoefficient", verbose);
  if (verbose)
  {
    std::cerr << " #### =============================================================================\n";
  }
  prefix = "swingLegTask.";
  if (verbose)
  {
    std::cerr << "\n #### Swing Leg Task:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, swingKp_, prefix + "kp", verbose);
  loadData::loadPtreeValue(pt, swingKd_, prefix + "kd", verbose);

  prefix = "baseAccelTask.";
  if (verbose)
  {
    std::cerr << "\n #### Base Accel(Tracking) Task:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, com_kp_, prefix + "kp", verbose);
  loadData::loadPtreeValue(pt, com_kd_, prefix + "kd", verbose);

  prefix = "baseHeightTask.";
  if (verbose)
  {
    std::cerr << "\n #### Base Height Task:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, baseHeightKp_, prefix + "kp", verbose);
  loadData::loadPtreeValue(pt, baseHeightKd_, prefix + "kd", verbose);
  prefix = "baseAngularTask.";
  if (verbose)
  {
    std::cerr << "\n #### Base Angular Task:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, baseAngularKp_, prefix + "kp", verbose);
  loadData::loadPtreeValue(pt, baseAngularKd_, prefix + "kd", verbose);
}

}  // namespace legged
