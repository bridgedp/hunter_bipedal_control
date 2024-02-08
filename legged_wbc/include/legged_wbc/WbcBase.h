//
// Created by qiayuan on 2022/7/1.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once

#include "legged_wbc/Task.h"

#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <legged_interface/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>

namespace legged
{
using namespace ocs2;
using namespace legged_robot;

// Ax -b = w
// Dx - f <= v
// w -> 0, v -> 0

// Decision Variables: x = [\dot u^T, 3*F(3)^T, \tau^T]^T , \dot u in ocal frame
class WbcBase
{
  using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
  using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;
  using Matrix6x = Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>;

public:
  WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
          const PinocchioEndEffectorKinematics& eeKinematics);

  virtual void loadTasksSetting(const std::string& taskFile, bool verbose);

  virtual vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured,
                          size_t mode, scalar_t period);
  void setCmdBodyPosVel(const vector_t& cmd_body_pos, const vector_t& cmd_body_vel)
  {
    cmd_body_pos_ = cmd_body_pos;
    cmd_body_vel_ = cmd_body_vel;
  }

  void setEarlyLateContact(const std::array<contact_flag_t, 2>& early_late_contact)
  {
    earlyLatecontact_ = early_late_contact;
  }

  void setFootPosVelAccDesired(const std::array<vector_t, 3>& footPosVelAccDesired)
  {
    footPosVelAccDesired_ = footPosVelAccDesired;
  }
  void setJointAccDesired(const vector_t& jointAccDesired)
  {
    jointAccDesired_ = jointAccDesired;
  }
  void setKpKd(scalar_t swingKp, scalar_t swingKd)
  {
    swingKp_ = swingKp;
    swingKd_ = swingKd;
  }
  size_t getContactForceSize()
  {
    return contact_force_size_;
  }
  void setStanceMode(bool stance_mode)
  {
    stance_mode_ = stance_mode;
  }

protected:
  void updateMeasured(const vector_t& rbdStateMeasured);
  void updateDesired(const vector_t& stateDesired, const vector_t& inputDesired);

  size_t getNumDecisionVars() const
  {
    return numDecisionVars_;
  }

  Task formulateFloatingBaseEomTask();
  Task formulateTorqueLimitsTask();
  Task formulateNoContactMotionTask();
  Task formulateFrictionConeTask();
  Task formulateBaseHeightMotionTask();
  Task formulateBaseAngularMotionTask();
  Task formulateBaseXYLinearAccelTask();
  Task formulateBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period);
  Task formulateSwingLegTask();
  Task formulateContactForceTask(const vector_t& inputDesired) const;

  void compensateFriction(vector_t& x);

  size_t numDecisionVars_;
  PinocchioInterface pinocchioInterfaceMeasured_, pinocchioInterfaceDesired_;
  CentroidalModelInfo info_;

  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;
  CentroidalModelPinocchioMapping mapping_;
  CentroidalModelRbdConversions rbdConversions_;

  vector_t qMeasured_, vMeasured_, inputLast_;
  matrix_t j_, dj_;
  Matrix6x base_j_, base_dj_;
  contact_flag_t contactFlag_{};
  size_t numContacts_{};

  vector_t torqueLimits_;
  scalar_t frictionCoeff_{}, swingKp_{}, swingKd_{};
  scalar_t baseHeightKp_{}, baseHeightKd_{};
  scalar_t baseAngularKp_{}, baseAngularKd_{};

  vector_t cmd_body_pos_;
  vector_t cmd_body_vel_;
  scalar_t com_kp_{}, com_kd_{};
  Vector6 basePoseDes_, baseVelocityDes_, baseAccelerationDes_;

  std::array<contact_flag_t, 2> earlyLatecontact_;

  std::vector<vector3_t> footPosDesired_, footVelDesired_;
  std::array<vector_t, 3> footPosVelAccDesired_;

  vector_t jointAccDesired_;
  size_t contact_force_size_ = 0;
  bool stance_mode_ = false;
};

}  // namespace legged
