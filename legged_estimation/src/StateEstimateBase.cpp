//
// Created by qiayuan on 2021/11/15.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "legged_estimation/StateEstimateBase.h"

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <legged_interface/common/Types.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace legged
{
using namespace legged_robot;

feet_array_t<std::vector<bool>> extractContactFlags(const std::vector<size_t>& phaseIDsStock)
{
  const size_t numPhases = phaseIDsStock.size();

  feet_array_t<std::vector<bool>> contactFlagStock;
  std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

  for (size_t i = 0; i < numPhases; i++)
  {
    const auto contactFlag = modeNumber2StanceLeg(phaseIDsStock[i]);
    for (size_t j = 0; j < contactFlag.size(); j++)
    {
      contactFlagStock[j][i] = contactFlag[j];
    }
  }
  return contactFlagStock;
}

StateEstimateBase::StateEstimateBase(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                     const PinocchioEndEffectorKinematics& eeKinematics)
  : pinocchioInterface_(std::move(pinocchioInterface))
  , info_(std::move(info))
  , eeKinematics_(eeKinematics.clone())
  , rbdState_(vector_t ::Zero(2 * info_.generalizedCoordinatesNum))
  , latestStanceposition_{}
{
  ros::NodeHandle nh;
  odomPub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(nh, "odom", 10));

  posePub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseWithCovarianceStamped>(nh, "pose", 10));

  pSCgZinvlast_.resize(info_.generalizedCoordinatesNum);
  pSCgZinvlast_.setZero();

  estContactforce_.resize(6 * 2 + 4);
  estContactforce_.fill(50);

  vMeasuredLast_.resize(info_.generalizedCoordinatesNum);
  vMeasuredLast_.fill(0);

  cmdTorque_.resize(info_.actuatedDofNum);
  cmdTorque_.setZero();

  earlyLateContactMsg_.data.resize(4, 0);
}

void StateEstimateBase::updateJointStates(const vector_t& jointPos, const vector_t& jointVel)
{
  rbdState_.segment(6, info_.actuatedDofNum) = jointPos;
  rbdState_.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum) = jointVel;
}

void StateEstimateBase::updateImu(const Eigen::Quaternion<scalar_t>& quat, const vector3_t& angularVelLocal,
                                  const vector3_t& linearAccelLocal, const matrix3_t& orientationCovariance,
                                  const matrix3_t& angularVelCovariance, const matrix3_t& linearAccelCovariance)
{
  quat_ = quat;
  angularVelLocal_ = angularVelLocal;
  linearAccelLocal_ = linearAccelLocal;
  orientationCovariance_ = orientationCovariance;
  angularVelCovariance_ = angularVelCovariance;
  linearAccelCovariance_ = linearAccelCovariance;

  vector3_t zyx = quatToZyx(quat) - zyxOffset_;
  vector3_t angularVelGlobal = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
      zyx, getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(quatToZyx(quat), angularVelLocal));
  updateAngular(zyx, angularVelGlobal);
}

void StateEstimateBase::updateAngular(const vector3_t& zyx, const vector_t& angularVel)
{
  rbdState_.segment<3>(0) = zyx;
  rbdState_.segment<3>(info_.generalizedCoordinatesNum) = angularVel;
}

void StateEstimateBase::updateLinear(const vector_t& pos, const vector_t& linearVel)
{
  rbdState_.segment<3>(3) = pos;
  rbdState_.segment<3>(info_.generalizedCoordinatesNum + 3) = linearVel;
}

void StateEstimateBase::publishMsgs(const nav_msgs::Odometry& odom)
{
  ros::Time time = odom.header.stamp;
  scalar_t publishRate = 200;
  if (lastPub_ + ros::Duration(1. / publishRate) < time)
  {
    lastPub_ = time;
    if (odomPub_->trylock())
    {
      odomPub_->msg_ = odom;
      odomPub_->unlockAndPublish();
    }
    if (posePub_->trylock())
    {
      posePub_->msg_.header = odom.header;
      posePub_->msg_.pose = odom.pose;
      posePub_->unlockAndPublish();
    }
  }
}

// ref: Contact Model Fusion for Event-Based Locomotion in Unstructured Terrains. Gerardo Bledt etc.
void StateEstimateBase::estContactForce(const ros::Duration& period)
{
  scalar_t dt = period.toSec();
  if (dt > 1)
    dt = 0.002;
  const scalar_t lamda = cutoffFrequency_;
  const scalar_t gama = exp(-lamda * dt);
  const scalar_t beta = (1 - gama) / (gama * dt);

  auto qMeasured_ = vector_t(info_.generalizedCoordinatesNum);
  auto vMeasured_ = vector_t(info_.generalizedCoordinatesNum);
  const auto& tauCmd = cmdTorque_;

  qMeasured_.head<3>() = rbdState_.segment<3>(3);
  qMeasured_.segment<3>(3) = rbdState_.head<3>();
  qMeasured_.tail(info_.actuatedDofNum) = rbdState_.segment(6, info_.actuatedDofNum);
  vMeasured_.head<3>() = rbdState_.segment<3>(info_.generalizedCoordinatesNum + 3);
  vMeasured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qMeasured_.segment<3>(3), rbdState_.segment<3>(info_.generalizedCoordinatesNum));
  vMeasured_.tail(info_.actuatedDofNum) = rbdState_.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();

  matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);
  s.block(0, 0, info_.actuatedDofNum, 6).setZero();
  s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();

  pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::updateFramePlacements(model, data);

  pinocchio::crba(model, data, qMeasured_);
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();

  pinocchio::getCoriolisMatrix(model, data);

  pinocchio::computeGeneralizedGravity(model, data, qMeasured_);

  vector_t p = data.M * vMeasured_;

  vector_t pSCg = beta * p + s.transpose() * tauCmd + data.C.transpose() * vMeasured_ - data.g;

  vector_t pSCg_z_inv = (1 - gama) * pSCg + gama * pSCgZinvlast_;
  pSCgZinvlast_ = pSCg_z_inv;

  estDisturbancetorque_ = beta * p - pSCg_z_inv;

  auto Jac_i = matrix_t(6, info_.generalizedCoordinatesNum);
  auto S_li = matrix_t(5, info_.generalizedCoordinatesNum);
  for (size_t i = 0; i < 2; ++i)
  {
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
    jac.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
    Jac_i = jac.template topRows<6>();
    S_li.setZero();
    int index = 0;
    if (i == 0)
      index = 0;
    else if (i == 1)
      index = 5;
    S_li.block<5, 5>(0, 6 + index) = Eigen::Matrix<scalar_t, 5, 5>::Identity();
    matrix56_t S_JT = S_li * Jac_i.transpose();
    vector5_t S_tau = S_li * estDisturbancetorque_;
    estContactforce_.segment<6>(6 * i) = S_JT.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(S_tau);
  }

  for (int i = 0; i < 2; i++)
  {
    estContactforce_(6 * 2 + i) = estContactforce_.segment<3>(6 * i).norm();
  }
  for (int i = 0; i < 2; i++)
  {
    estContactforce_(6 * 2 + 2 + i) = estContactforce_.segment<6>(6 * i).norm();
  }
}

contact_flag_t StateEstimateBase::estContactState(const scalar_t& time)
{
  contact_flag_t contact_state = cmdContactflag_;
  for (int i = 0; i < info_.numThreeDofContacts; i++)
  {
    const auto start_time = StartStopTime4Legs_[i].front();
    const auto stop_time = StartStopTime4Legs_[i].back();
    const auto period = stop_time - start_time;
    if (!cmdContactflag_[i] && (time - start_time > 0.75 * period))  // swing legs
    {
      contact_state[i] = (estContactforce_(6 * (i % 2) + 2) > contactThreshold_) ? true : false;
    }
    if (cmdContactflag_[i] && (time - start_time < 0.25 * period))  // stance legs
    {
      contact_state[i] = (estContactforce_(6 * (i % 2) + 2) > contactThreshold_) ? true : false;
    }
  }
  return std::move(contact_state);
}

void StateEstimateBase::earlyContactDetection(const ModeSchedule& modeSchedule, scalar_t current_time)
{
  auto& modeSequence = modeSchedule.modeSequence;
  auto& eventTimes = modeSchedule.eventTimes;
  auto con_seq = extractContactFlags(modeSequence);

  // first, find where to insert by time sequence
  auto time_insert_it = std::lower_bound(eventTimes.begin(), eventTimes.end(), current_time);
  // "id" is the new element's index
  const size_t id = std::distance(eventTimes.begin(), time_insert_it);

  // reset early contact flag
  for (int leg = 0; leg < 4; leg++)
  {
    earlyLatecontact_[0][leg] = false;
  }

  bool insert_flag = false;
  size_array_t insert_legs;
  // check if there is early contact, and find each leg
  for (int leg = 0; leg < 4; leg++)
  {
    if (!con_seq[leg][id] && contactFlag_[leg])
    {
      insert_flag = true;
      insert_legs.push_back(leg);
    }
  }

  // mask the early contact feet's contact sequence
  if (insert_flag)
  {
    if (abs(eventTimes.at(id) - current_time) > 0.001 && abs(eventTimes.at(id - 1) - current_time) > 0.001)
    {
      size_array_t filtered_insert_legs;
      // check if leg just start swing, if do, remove that leg
      for (auto leg : insert_legs)
      {
        // find swing start time
        scalar_t start_time = current_time;
        for (size_t i = id; i > 0; i--)
        {
          if (!con_seq[leg][i])
            start_time = eventTimes[i - 1];
          else
            break;
        }
        // find swing stop time
        scalar_t stop_time = current_time;
        for (size_t i = id; i < eventTimes.size(); i++)
        {
          if (!con_seq[leg][i])
            stop_time = eventTimes[i];
          else
            break;
        }
        const scalar_t length = stop_time - start_time;
        // if just start swing, or very close to finish, pass that leg
        if (current_time - start_time > 0.75 * length && stop_time - current_time > 0.009)
        {
          earlyLatecontact_[0][leg] = true;
        }
      }
    }
  }
}

void StateEstimateBase::lateContactDetection(const ModeSchedule& modeSchedule, scalar_t current_time)
{
  auto& modeSequence = modeSchedule.modeSequence;
  auto& eventTimes = modeSchedule.eventTimes;
  auto con_seq = extractContactFlags(modeSequence);

  estConHistory_.push_front(std::pair<scalar_t, contact_flag_t>{ current_time, contactFlag_ });
  if (estConHistory_.size() > 10)
    estConHistory_.pop_back();

  // # late contact handle
  // ## find close cmd stance foot
  // ### get swing start time
  auto time_insert_it = std::lower_bound(eventTimes.begin(), eventTimes.end(), current_time);
  // "id" is the new element's index
  const size_t id = std::distance(eventTimes.begin(), time_insert_it);

  // reset late contact flag
  for (int leg = 0; leg < 4; leg++)
  {
    earlyLatecontact_[1][leg] = false;
  }

  size_array_t stance_legs;
  size_array_t swing_legs;
  for (int leg = 0; leg < 4; leg++)
  {
    if (con_seq[leg][id])  // cmd stance leg
      stance_legs.push_back(leg);
    else  // cmd swing leg
      swing_legs.push_back(leg);
  }
  // check stance leg first, and only delay once
  bool delayed_flag = false;
  // leg whether need to swing down
  feet_array_t<bool> leg_swing_down_flags{ false, false };
  for (auto leg : stance_legs)  // for cmd stance leg
  {
    // find stance start time
    scalar_t start_time = current_time;
    for (size_t i = id; i > 0; i--)
    {
      if (con_seq[leg][i])
        start_time = eventTimes[i - 1];
      else
        break;
    }
    // if just finish swing cmd and no contact detected
    if (current_time - start_time < 0.04 && !contactFlag_[leg])
    {
      bool back_check = true;
      for (auto histroy_con : estConHistory_)
      {
        if (histroy_con.first >= start_time)
        {
          if (histroy_con.second[leg])
          {
            back_check = false;
          }
        }
      }
      if (back_check)
      {
        earlyLatecontact_[1][leg] = true;
      }
    }
  }
}

void StateEstimateBase::loadSettings(const std::string& taskFile, bool verbose)
{
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "contactForceEsimation.";
  if (verbose)
  {
    std::cerr << "\n #### contactForceEsimation:";
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::loadPtreeValue(pt, cutoffFrequency_, prefix + "cutoffFrequency", verbose);
  loadData::loadPtreeValue(pt, contactThreshold_, prefix + "contactThreshold", verbose);
}

}  // namespace legged
