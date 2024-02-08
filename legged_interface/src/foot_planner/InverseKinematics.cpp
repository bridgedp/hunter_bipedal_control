/********************************************************************************
Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include "legged_interface/foot_planner/InverseKinematics.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2
{
namespace legged_robot
{
vector5_t InverseKinematics::computeIK(vector_t init_q, int leg, vector3_t des_foot_linear_xyz,
                                       vector3_t des_foot_eular_zyx)
{
  const int index = leg2index(leg);
  init_q.segment<5>(6 + index) = computeTranslationIK(init_q, leg, des_foot_linear_xyz);
  return computeRotationIK(init_q, leg, des_foot_eular_zyx);
}

vector5_t InverseKinematics::computeIK(vector_t init_q, int leg, vector3_t des_foot_linear_xyz,
                                       matrix3_t des_foot_R_des)
{
  const int index = leg2index(leg);
  init_q.segment<5>(6 + index) = computeTranslationIK(init_q, leg, des_foot_linear_xyz);
  return computeRotationIK(init_q, leg, des_foot_R_des);
}

vector5_t InverseKinematics::computeTranslationIK(vector_t init_q, int leg, vector3_t des_foot_linear_xyz)
{
  const scalar_t err_tol = 0.01;
  const scalar_t converage_tol = 0.001;
  const scalar_t damp = 1e-6;
  const scalar_t dt = 0.7;
  const int max_it = 5;
  const bool verbose = false;

  const auto& des_foot_p = des_foot_linear_xyz;
  const auto& model = pinocchio_interface_->getModel();
  auto& data = pinocchio_interface_->getData();
  auto FRAME_ID = info_->endEffectorFrameIndices[leg];
  int index = leg2index(leg);

  vector5_t leg_q_min = model.lowerPositionLimit.segment<5>(6 + index);
  vector5_t leg_q_max = model.upperPositionLimit.segment<5>(6 + index);

  matrix35_t Jac_i;
  Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> Jac;
  Jac.setZero(6, model.nv);
  vector_t v(model.nv);
  v.setZero();

  pinocchio::framesForwardKinematics(model, data, init_q);
  const vector3_t init_foot_pos = data.oMf[FRAME_ID].translation();

  vector3_t err = init_foot_pos - des_foot_p;
  scalar_t last_err_norm = err.norm();

  if (last_err_norm < err_tol)
  {
    if (verbose)
      printf("[translation IK] small err at init, err norm: %f\n", last_err_norm);
  }
  else
  {
    int i = 0;
    while (true)
    {
      pinocchio::computeFrameJacobian(model, data, init_q, FRAME_ID, pinocchio::LOCAL_WORLD_ALIGNED, Jac);
      Jac_i = Jac.block<3, 5>(0, 6 + index);
      qr_.compute(Jac_i);
      vector5_t v_i = -qr_.solve(err);
      v.segment<5>(6 + index) = v_i;
      vector_t new_q = pinocchio::integrate(model, init_q, v * dt);

      for (int i = 0; i < 5; i++)
      {
        new_q[6 + index + i] = std::max(leg_q_min[i], new_q[6 + index + i]);
        new_q[6 + index + i] = std::min(leg_q_max[i], new_q[6 + index + i]);
      }

      pinocchio::framesForwardKinematics(model, data, new_q);
      vector3_t cur_foot_pos = data.oMf[FRAME_ID].translation();
      err = cur_foot_pos - des_foot_p;

      scalar_t err_norm = err.norm();

      if (err_norm > last_err_norm)
      {
        if (verbose)
          printf("[translation IK] local min at %d, last err norm: %f, new err norm: %f\n", i, last_err_norm, err_norm);
        break;
      }
      if (abs(err_norm - last_err_norm) < converage_tol)
      {
        if (verbose)
          printf("[translation IK] converage at %d, new err norm: %f\n", i, err_norm);
        break;
      }
      last_err_norm = err_norm;
      init_q = new_q;

      if (err_norm < err_tol)
      {
        if (verbose)
          printf("[translation IK] small err at %d, err norm: %f\n", i, err_norm);
        break;
      }
      i++;
      if (i >= max_it)
      {
        if (verbose)
          printf("[translation IK] max iter at %d, err norm: %f\n", i, err_norm);
        break;
      }
    }
  }

  return std::move(init_q.segment<5>(6 + index));
}

vector5_t InverseKinematics::computeRotationIK(vector_t init_q, int leg, vector3_t des_foot_eular_zyx)
{
  matrix3_t R_des = getRotationMatrixFromZyxEulerAngles(des_foot_eular_zyx);
  return computeRotationIK(init_q, leg, R_des);
}

vector5_t InverseKinematics::computeRotationIK(vector_t init_q, int leg, matrix3_t des_foot_R_des)
{
  const scalar_t err_tol = 0.01;
  const scalar_t converage_tol = 0.001;
  const scalar_t damp = 1e-6;
  const scalar_t dt = 0.7;
  const int max_it = 5;
  const bool verbose = false;

  const auto& model = pinocchio_interface_->getModel();
  auto& data = pinocchio_interface_->getData();

  auto FRAME_ID = info_->endEffectorFrameIndices[leg];

  int index = leg2index(leg);

  vector5_t leg_q_min = model.lowerPositionLimit.segment<5>(6 + index);
  vector5_t leg_q_max = model.upperPositionLimit.segment<5>(6 + index);

  matrix35_t Jac_i;
  matrix35_t Jac_linear_i;
  Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> Jac;
  Jac.setZero(6, model.nv);
  vector_t v(model.nv);
  v.setZero();

  pinocchio::framesForwardKinematics(model, data, init_q);
  matrix3_t R_cur = data.oMf[FRAME_ID].rotation();

  vector3_t err = pinocchio::log3(des_foot_R_des.transpose() * R_cur);
  scalar_t last_err_norm = err.norm();

  if (last_err_norm < err_tol)
  {
    if (verbose)
      printf("[rotation IK] small err at init, err norm: %f\n", last_err_norm);
  }
  else
  {
    int i = 0;
    while (true)
    {
      pinocchio::computeFrameJacobian(model, data, init_q, FRAME_ID, pinocchio::LOCAL, Jac);
      Jac_i = Jac.block<3, 5>(3, 6 + index);
      Jac_linear_i = Jac.block<3, 5>(0, 6 + index);
      matrix_t null_space = Jac_linear_i.fullPivLu().kernel();

      qr_.compute(Jac_i * null_space);
      vector_t v_i_n = qr_.solve(err);
      vector5_t v_i = -null_space * v_i_n;
      v.segment<5>(6 + index) = v_i;
      vector_t new_q = pinocchio::integrate(model, init_q, v * dt);

      for (int i = 0; i < 5; i++)
      {
        new_q[6 + index + i] = std::max(leg_q_min[i], new_q[6 + index + i]);
        new_q[6 + index + i] = std::min(leg_q_max[i], new_q[6 + index + i]);
      }

      pinocchio::framesForwardKinematics(model, data, new_q);
      R_cur = data.oMf[FRAME_ID].rotation();
      err = pinocchio::log3(des_foot_R_des.transpose() * R_cur);

      scalar_t err_norm = err.norm();

      if (err_norm > last_err_norm)
      {
        if (verbose)
          printf("[rotation IK] local min at %d, last err norm: %f, new err norm: %f\n", i, last_err_norm, err_norm);
        break;
      }
      if (abs(err_norm - last_err_norm) < converage_tol)
      {
        if (verbose)
          printf("[rotation IK] converage at %d, last err norm: %f, new err norm: %f\n", i, last_err_norm, err_norm);
        break;
      }
      last_err_norm = err_norm;
      init_q = new_q;

      if (err_norm < err_tol)
      {
        if (verbose)
          printf("[rotation IK] small err at %d, err norm: %f\n", i, err_norm);
        break;
      }
      i++;
      if (i >= max_it)
      {
        if (verbose)
          printf("[rotation IK] max iter at %d, err norm: %f\n", i, err_norm);
        break;
      }
    }
  }
  return std::move(init_q.segment<5>(6 + index));
}

vector5_t InverseKinematics::computeDIK(vector_t q, int leg, vector3_t foot_linear_vel, vector3_t foot_angular_vel)
{
  vector5_t joint_vel;
  const auto& model = pinocchio_interface_->getModel();
  auto& data = pinocchio_interface_->getData();
  auto FRAME_ID = info_->endEffectorFrameIndices[leg];
  Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> Jac;
  Jac.setZero(6, model.nv);
  pinocchio::framesForwardKinematics(model, data, q);
  pinocchio::computeFrameJacobian(model, data, q, FRAME_ID, pinocchio::LOCAL_WORLD_ALIGNED, Jac);
  matrix65_t Jac_i;
  const int index = leg2index(leg);
  Jac_i = Jac.block<6, 5>(0, 6 + index);
  qr_.compute(Jac_i);
  vector6_t foot_vel;
  foot_vel << foot_linear_vel, foot_angular_vel;
  joint_vel = qr_.solve(foot_vel);
  return std::move(joint_vel);
}

feet_array_t<vector3_t> InverseKinematics::computeFootPos(const vector_t& state)
{
  const auto& model = pinocchio_interface_->getModel();
  auto& data = pinocchio_interface_->getData();
  const vector_t& init_q = state.tail(info_->generalizedCoordinatesNum);
  pinocchio::framesForwardKinematics(model, data, init_q);
  feet_array_t<vector3_t> feet_pos;
  for (int leg = 0; leg < feet_pos.size(); leg++)
  {
    auto FRAME_ID = info_->endEffectorFrameIndices[leg];
    int index = leg2index(leg);
    feet_pos[leg] = data.oMf[FRAME_ID].translation();
  }
  return std::move(feet_pos);
}

}  // namespace legged_robot
}  // namespace ocs2
