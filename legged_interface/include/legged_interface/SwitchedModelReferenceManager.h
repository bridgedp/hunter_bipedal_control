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

#pragma once

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include "legged_interface/gait/GaitSchedule.h"
#include "legged_interface/gait/MotionPhaseDefinition.h"

#include "legged_interface/foot_planner/SwingTrajectoryPlanner.h"

#include "ocs2_core/thread_support/BufferedValue.h"
#include "ocs2_pinocchio_interface/PinocchioInterface.h"
#include "ocs2_centroidal_model/CentroidalModelInfo.h"

#include <atomic>
#include <deque>
#include <utility>

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <ocs2_core/misc/Benchmark.h>
#include "legged_interface/foot_planner/InverseKinematics.h"

namespace ocs2
{
namespace legged_robot
{
/**
 * Manages the ModeSchedule and the TargetTrajectories for switched model.
 */
class SwitchedModelReferenceManager : public ReferenceManager
{
public:
  SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                PinocchioInterface pinocchioInterface, CentroidalModelInfo info);

  ~SwitchedModelReferenceManager() override = default;

  void setModeSchedule(const ModeSchedule& modeSchedule) override;

  void setPitchRoll(const scalar_t& pitch, const scalar_t& roll)
  {
    pitch_ = pitch;
    roll_ = roll;
  }

  void setLatestStancePosition(feet_array_t<vector3_t> latest_stance_position)
  {
    latestStancePosition_.setBuffer(latest_stance_position);
  }

  void setFeetBias(feet_array_t<vector3_t> feet_bias)
  {
    feetBiasBuffer_.setBuffer(feet_bias);
  }

  void setEstContactFlag(contact_flag_t est_contact_flag)
  {
    estContactFlagBuffer_.setBuffer(est_contact_flag);
  }

  void setPinocchioInterface(PinocchioInterface pinocchioInterface)
  {
    pinocchioInterface_ = std::move(pinocchioInterface);
  }

  contact_flag_t getContactFlags(scalar_t time) const;

  vector_t getCmdBodyVel()
  {
    velCmdOutBuf_.updateFromBuffer();
    return velCmdOutBuf_.get();
  }

  const std::shared_ptr<GaitSchedule>& getGaitSchedule()
  {
    return gaitSchedulePtr_;
  }

  const std::shared_ptr<SwingTrajectoryPlanner>& getSwingTrajectoryPlanner()
  {
    return swingTrajectoryPtr_;
  }

protected:
  void calculateVelAbs(TargetTrajectories& targetTrajectories);

  void walkGait(scalar_t body_height, scalar_t initTime, scalar_t finalTime, ModeSchedule& modeSchedule);

  void trotGait(scalar_t body_height, scalar_t initTime, scalar_t finalTime);

  void calculateJointRef(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                         TargetTrajectories& targetTrajectories);

  void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                        TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) override;

  scalar_t findInsertModeSequenceTemplateTimer(ModeSchedule& modeSchedule, scalar_t current_time);

  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
  std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;

  std::atomic<scalar_t> pitch_;
  std::atomic<scalar_t> roll_;
  BufferedValue<contact_flag_t> estContactFlagBuffer_;

  BufferedValue<feet_array_t<vector3_t>> latestStancePosition_;
  BufferedValue<feet_array_t<vector3_t>> feetBiasBuffer_;

  ros::Subscriber velCmdSub_;
  ros::Subscriber gaitTypeSub_;

  std_msgs::Float64MultiArray earlyLateContactMsg_;

  BufferedValue<vector_t> velCmdInBuf_;
  BufferedValue<vector_t> velCmdOutBuf_;

  std::atomic_int32_t gaitType_{ 0 };
  int gaitLevel_{ 0 };

  std::deque<scalar_t> velAbsHistory_;

  scalar_t stanceTime_;
  scalar_t velAvg_;
  scalar_t velAbs_;

  PinocchioInterface pinocchioInterface_;
  CentroidalModelInfo info_;

  benchmark::RepeatedTimer ikTimer_;

  vector_t defaultJointState_;
  InverseKinematics inverseKinematics_;
};

}  // namespace legged_robot
}  // namespace ocs2
