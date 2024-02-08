/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include "legged_interface/SwitchedModelReferenceManager.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <numeric>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <iostream>
#include <iomanip>

namespace ocs2
{
namespace legged_robot
{
std::vector<scalar_t> stance_times{ 0.0, 0.5 };
std::vector<size_t> stance_modes{ 3 };
ModeSequenceTemplate stance(stance_times, stance_modes);

std::vector<scalar_t> trot_times{ 0.0, 0.3, 0.6 };
std::vector<size_t> trot_modes{ 2, 1 };
ModeSequenceTemplate trot(trot_times, trot_modes);


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwitchedModelReferenceManager::SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                             std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                                             PinocchioInterface pinocchioInterface,
                                                             CentroidalModelInfo info)
  : ReferenceManager(TargetTrajectories(), ModeSchedule())
  , gaitSchedulePtr_(std::move(gaitSchedulePtr))
  , swingTrajectoryPtr_(std::move(swingTrajectoryPtr))
  , latestStancePosition_(std::move(feet_array_t<vector3_t>{}))
  , feetBiasBuffer_(std::move(feet_array_t<vector3_t>{}))
  , estContactFlagBuffer_(std::move(contact_flag_t{}))
  , pinocchioInterface_(std::move(pinocchioInterface))
  , info_(std::move(info))
  , velCmdInBuf_(std::move(vector_t::Zero(6)))
  , velCmdOutBuf_(std::move(vector_t::Zero(6)))
{
  pitch_ = 0.0;
  roll_ = 0.0;
  feet_array_t<vector3_t> feet_bias{};

  auto nh = ros::NodeHandle();

  earlyLateContactMsg_.data.resize(info_.numThreeDofContacts, 0);

  // cmd_vel subscriber
  auto cmdVelCallback = [this](const geometry_msgs::Twist::ConstPtr& msg) {
    vector_t cmdVel = vector_t::Zero(6);
    cmdVel[0] = msg->linear.x;
    cmdVel[1] = msg->linear.y;
    cmdVel[2] = msg->linear.z;
    cmdVel[3] = msg->angular.z;
    velCmdInBuf_.setBuffer(cmdVel);
    velCmdOutBuf_.setBuffer(cmdVel);
  };

  velCmdSub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel_filtered", 1, cmdVelCallback);

  auto gait_type_callback = [this](const std_msgs::Int32::ConstPtr& msg) { gaitType_ = msg->data; };
  gaitTypeSub_ = nh.subscribe<std_msgs::Int32>("/gait_type", 1, gait_type_callback);
  std::string referenceFile;
  nh.getParam("/referenceFile", referenceFile);
  defaultJointState_.resize(info_.actuatedDofNum);
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", defaultJointState_);
  inverseKinematics_.setParam(std::make_shared<PinocchioInterface>(pinocchioInterface_),
                              std::make_shared<CentroidalModelInfo>(info_));

  std::cout.setf(std::ios::fixed);
  std::cout.precision(6);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManager::setModeSchedule(const ModeSchedule& modeSchedule)
{
  ReferenceManager::setModeSchedule(modeSchedule);
  gaitSchedulePtr_->setModeSchedule(modeSchedule);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
contact_flag_t SwitchedModelReferenceManager::getContactFlags(scalar_t time) const
{
  return modeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                     TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule)
{
  if (targetTrajectories.size() == 1)
  {
    targetTrajectories.timeTrajectory.push_back(finalTime);
    targetTrajectories.stateTrajectory.push_back(targetTrajectories.stateTrajectory.front());
    targetTrajectories.inputTrajectory.push_back(targetTrajectories.inputTrajectory.front());
  }
  const auto timeHorizon = finalTime - initTime;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);
  scalar_t body_height = targetTrajectories.stateTrajectory[0](8);

  estContactFlagBuffer_.updateFromBuffer();
  const auto& est_con = estContactFlagBuffer_.get();

  const scalar_t terrainHeight = 0.0;

  calculateVelAbs(targetTrajectories);

  if (gaitType_ == 0)
  {
    walkGait(body_height, initTime, finalTime, modeSchedule);
  }
  else if (gaitType_ == 2)
  {
    trotGait(body_height, initTime, finalTime);
  }

  swingTrajectoryPtr_->setGaitLevel(gaitLevel_);
  swingTrajectoryPtr_->setBodyVelCmd(velCmdInBuf_.get());
  swingTrajectoryPtr_->setCurrentFeetPosition(inverseKinematics_.computeFootPos(initState));
  swingTrajectoryPtr_->update(modeSchedule, targetTrajectories, initTime);

  calculateJointRef(initTime, finalTime, initState, targetTrajectories);
}

scalar_t SwitchedModelReferenceManager::findInsertModeSequenceTemplateTimer(ModeSchedule& modeSchedule,
                                                                            scalar_t current_time)
{
  auto& modeSequence = modeSchedule.modeSequence;
  auto& eventTimes = modeSchedule.eventTimes;

  const auto time_insert_it = std::lower_bound(eventTimes.begin(), eventTimes.end(), current_time);
  const size_t id = std::distance(eventTimes.begin(), time_insert_it);

  return eventTimes[id];
}

void SwitchedModelReferenceManager::walkGait(scalar_t body_height, scalar_t initTime, scalar_t finalTime,
                                             ModeSchedule& modeSchedule)
{
  if (velAvg_ <= 0.02)
  {
    if (gaitLevel_ != 0)
    {
      printf("start to stance\n");
      auto inserTimer = findInsertModeSequenceTemplateTimer(modeSchedule, initTime);
      gaitSchedulePtr_->insertModeSequenceTemplate(stance, inserTimer, finalTime);
      gaitLevel_ = 0;
    }
  }
  else if (velAvg_ > 0.03 && velAvg_ < 0.4)
  {
    if (gaitLevel_ != 1)
    {
      printf("start to trot\n");
      auto inserTimer = findInsertModeSequenceTemplateTimer(modeSchedule, initTime);
      gaitSchedulePtr_->insertModeSequenceTemplate(trot, inserTimer, finalTime);
      gaitLevel_ = 1;
    }
  }
  else if (velAvg_ >= 0.4)
  {
    if (gaitLevel_ != 3)
    {
      printf("start to flying trot\n");
      auto inserTimer = findInsertModeSequenceTemplateTimer(modeSchedule, initTime);
      gaitLevel_ = 3;
    }
  }
}

void SwitchedModelReferenceManager::trotGait(scalar_t body_height, scalar_t initTime, scalar_t finalTime)
{
  if (gaitLevel_ != 1)
  {
    printf("start to trot\n");
    gaitSchedulePtr_->insertModeSequenceTemplate(trot, initTime + 0.2, finalTime);
    gaitLevel_ = 1;
  }
}

void SwitchedModelReferenceManager::calculateVelAbs(TargetTrajectories& targetTrajectories)
{
  velCmdInBuf_.updateFromBuffer();
  auto vel_cmd = velCmdInBuf_.get();
  auto vel_est = swingTrajectoryPtr_->getBodyVelWorld();
  vel_est = targetTrajectories.stateTrajectory[0].segment(0, 6);
  vector3_t angular = targetTrajectories.stateTrajectory[0].segment(9, 3);
  vel_cmd.head(3) = getRotationMatrixFromZyxEulerAngles(angular) * vel_cmd.head(3);
  vel_cmd(2) = 0;
  vel_cmd(3) = vel_cmd(3) / 3.0;
  auto vel_cmd_abs = vel_cmd.head(4).norm();
  vel_est(2) = 0;
  vel_est(3) = vel_est(3) / 3.0;
  auto vel_est_abs = vel_est.head(4).norm();
  velAbs_ = (0.5 * vel_cmd.head(4) + 0.5 * vel_est.head(4)).norm();  // vel_est has great error
  velAbsHistory_.push_front(velAbs_);
  while (velAbsHistory_.size() > 50)
    velAbsHistory_.pop_back();
  velAvg_ = std::accumulate(velAbsHistory_.begin(), velAbsHistory_.end(), 0.0) / velAbsHistory_.size();
  stanceTime_ = std::max(0.225 / velAvg_, 0.15);
}

void SwitchedModelReferenceManager::calculateJointRef(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                      TargetTrajectories& targetTrajectories)
{
  vector3_t euler_zyx = initState.segment<3>(6 + 3);
  matrix3_t world2body_rotation = getRotationMatrixFromZyxEulerAngles(euler_zyx);
  matrix3_t body2foot_rotation = getRotationMatrixFromZyxEulerAngles(vector3_t(0, 0, 0));
  matrix3_t R_des = world2body_rotation * body2foot_rotation;
  if (targetTrajectories.size() <= 1)
    return;
  auto q_ref = vector_t(info_.generalizedCoordinatesNum);

  const scalar_t step = 0.15;
  int sample_size = floor((finalTime - initTime) / step) + 1;
  if (sample_size <= 2)
    return;
  vector_t Ts = vector_t::LinSpaced(sample_size, initTime, finalTime);
  const auto old_target_trajectories = targetTrajectories;

  targetTrajectories.timeTrajectory.resize(sample_size);
  targetTrajectories.stateTrajectory.resize(sample_size);
  targetTrajectories.inputTrajectory.resize(sample_size);

  for (int i = 0; i < sample_size; i++)
  {
    targetTrajectories.timeTrajectory[i] = Ts[i];
    targetTrajectories.stateTrajectory[i] = old_target_trajectories.getDesiredState(Ts[i]);
    targetTrajectories.inputTrajectory[i] = old_target_trajectories.getDesiredInput(Ts[i]);
  }
  const auto& joint_num = info_.actuatedDofNum;
  targetTrajectories.stateTrajectory[0].segment(6 + 6, joint_num) = defaultJointState_;

  const int feet_num = 2;
  for (int i = 0; i < sample_size; i++)
  {
    q_ref.head<6>() = targetTrajectories.stateTrajectory[i].segment<6>(6);
    q_ref.segment(6, joint_num) = targetTrajectories.stateTrajectory[std::max(i - 1, 0)].segment(6 + 6, joint_num);
    vector3_t des_foot_p;
    for (int leg = 0; leg < feet_num; leg++)
    {
      int index = InverseKinematics::leg2index(leg);
      des_foot_p.x() = swingTrajectoryPtr_->getXpositionConstraint(leg, Ts[i]);
      des_foot_p.y() = swingTrajectoryPtr_->getYpositionConstraint(leg, Ts[i]);
      des_foot_p.z() = swingTrajectoryPtr_->getZpositionConstraint(leg, Ts[i]);
      ikTimer_.startTimer();
      targetTrajectories.stateTrajectory[i].segment<5>(12 + index) =
          inverseKinematics_.computeIK(q_ref, leg, des_foot_p, R_des);
      ikTimer_.endTimer();
    }
  }
}

}  // namespace legged_robot
}  // namespace ocs2
