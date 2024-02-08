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

#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_core/reference/TargetTrajectories.h>

#include <legged_interface/common/Types.h>
#include "legged_interface/foot_planner/SplineCpg.h"
#include "legged_interface/foot_planner/MultiCubicSpline.h"

#include "ocs2_core/thread_support/BufferedValue.h"

#include "ros/ros.h"

#include <dynamic_reconfigure/server.h>
// #include "legged_interface/SwingTrajectoryPlannerConfig.h"

namespace ocs2
{
namespace legged_robot
{
class SwingTrajectoryPlanner
{
public:
  struct Config
  {
    scalar_t liftOffVelocity = 0.0;
    scalar_t touchDownVelocity = 0.0;
    scalar_t swingHeight = 0.1;
    scalar_t swingTimeScale = 0.15;  // swing phases shorter than this time will be scaled down in height and velocity
    scalar_t feet_bias_x1 = 0;
    scalar_t feet_bias_x2 = 0;
    scalar_t feet_bias_y = 0;
    scalar_t feet_bias_z = 0;
    scalar_t next_position_z = 0.02;
  };

  SwingTrajectoryPlanner(Config config);

  void update(const ModeSchedule& modeSchedule, const TargetTrajectories& targetTrajectories, scalar_t initTime);

  void setFeetBias(const feet_array_t<vector3_t>& feet_bias)
  {
    feet_bias_ = feet_bias;
  }
  void setLegSwingDownFlags(const feet_array_t<bool>& leg_swing_down_flags, scalar_t delayed_time)
  {
    leg_swing_down_flags_ = leg_swing_down_flags;
    delayed_time_ = delayed_time;
  }
  void setBodyVelWorld(vector_t body_vel)
  {
    body_vel_world_buf_.setBuffer(body_vel);
  }
  void setBodyVelCmd(vector_t body_vel_cmd)
  {
    body_vel_cmd_ = body_vel_cmd;
  }
  void setGaitLevel(int gait_level)
  {
    gaitLevel_ = gait_level;
  }
  void setCurrentFeetPosition(const feet_array_t<vector3_t>& current_feet_position)
  {
    current_feet_position_buf_.setBuffer(current_feet_position);
  }
  void setCurrentFeetPosition(const vector_t& current_feet_position)
  {
    feet_array_t<vector3_t> feet_pos;
    for (int i = 0; i < numFeet_; i++)
      feet_pos[i] = current_feet_position.segment<3>(3 * i);
    current_feet_position_buf_.setBuffer(feet_pos);
  }

  vector_t getBodyVelWorld()
  {
    body_vel_world_buf_.updateFromBuffer();
    return body_vel_world_buf_.get();
  }

  scalar_t getXvelocityConstraint(size_t leg, scalar_t time) const;
  scalar_t getYvelocityConstraint(size_t leg, scalar_t time) const;
  scalar_t getZvelocityConstraint(size_t leg, scalar_t time) const;

  scalar_t getXpositionConstraint(size_t leg, scalar_t time) const;
  scalar_t getYpositionConstraint(size_t leg, scalar_t time) const;
  scalar_t getZpositionConstraint(size_t leg, scalar_t time) const;

  std::array<scalar_t, 2> getSwingStartStopTime(size_t leg, scalar_t time) const;
  feet_array_t<std::array<scalar_t, 2>> threadSaftyGetStartStopTime(scalar_t time);

  feet_array_t<std::array<vector_t, 6>> threadSaftyGetPosVel(const vector_t& time_sample);

  /**
   * Extracts for each leg the contact sequence over the motion phase sequence.
   * @param phaseIDsStock
   * @return contactFlagStock
   */
  feet_array_t<std::vector<bool>> extractContactFlags(const std::vector<size_t>& phaseIDsStock) const;

private:
  vector3_t calNextFootPos(int feet, scalar_t current_time, scalar_t stop_time, scalar_t next_middle_time,
                           const vector_t& next_middle_body_pos, const vector_t& current_body_pos,
                           const vector3_t& current_body_vel);

  void genSwingTrajs(int feet, scalar_t current_time, scalar_t start_time, scalar_t stop_time,
                     const vector3_t& start_pos, const vector3_t& stop_pos);

  void genSwingTrajsJoyControl(int feet, scalar_t start_time, scalar_t stop_time,
                               const vector3_t& current_feet_position, const vector3_t& swing_cmd);

  scalar_t swingTrajectoryScaling(scalar_t startTime, scalar_t finalTime, scalar_t swingTimeScale)
  {
    return std::min(1.0, (finalTime - startTime) / swingTimeScale);
  }

  /**
   * Finds the take-off and touch-down times indices for a specific leg.
   *
   * @param index
   * @param contactFlagStock
   * @return {The take-off time index for swing legs, touch-down time index for swing legs}
   */
  static std::pair<int, int> findIndex(size_t index, const std::vector<bool>& contactFlagStock);

  /**
   * based on the input phaseIDsStock finds the start subsystem and final subsystem of the swing
   * phases of the a foot in each subsystem.
   *
   * startTimeIndexStock: eventTimes[startTimesIndex] will be the take-off time for the requested leg.
   * finalTimeIndexStock: eventTimes[finalTimesIndex] will be the touch-down time for the requested leg.
   *
   * @param [in] footIndex: Foot index
   * @param [in] phaseIDsStock: The sequence of the motion phase IDs.
   * @param [in] contactFlagStock: The sequence of the contact status for the requested leg.
   * @return { startTimeIndexStock, finalTimeIndexStock}
   */
  static std::pair<std::vector<int>, std::vector<int>> updateFootSchedule(const std::vector<bool>& contactFlagStock);

  /**
   * Check if event time indices are valid
   * @param leg
   * @param index : phase index
   * @param startIndex : liftoff event time index
   * @param finalIndex : touchdown event time index
   * @param phaseIDsStock : mode sequence
   */
  static void checkThatIndicesAreValid(int leg, int index, int startIndex, int finalIndex,
                                       const std::vector<size_t>& phaseIDsStock);

  const Config config_;
  size_t numFeet_;

  feet_array_t<std::vector<MultiCubicSpline>> feetXTrajs_;
  feet_array_t<std::vector<MultiCubicSpline>> feetYTrajs_;
  feet_array_t<std::vector<MultiCubicSpline>> feetZTrajs_;

  BufferedValue<feet_array_t<std::vector<MultiCubicSpline>>> feetXTrajsBuf_;
  BufferedValue<feet_array_t<std::vector<MultiCubicSpline>>> feetYTrajsBuf_;
  BufferedValue<feet_array_t<std::vector<MultiCubicSpline>>> feetZTrajsBuf_;
  BufferedValue<std::vector<scalar_t>> footTrajsEventsBuf_;
  BufferedValue<feet_array_t<std::vector<std::array<scalar_t, 2>>>> StartStopTimeBuf_;

  feet_array_t<std::vector<scalar_t>> feetTrajsEvents_;
  feet_array_t<std::vector<std::array<scalar_t, 2>>> startStopTime_;
  feet_array_t<vector3_t> feet_bias_;

  feet_array_t<bool> leg_swing_down_flags_;
  scalar_t delayed_time_{ 0 };
  BufferedValue<vector_t> body_vel_world_buf_;

  vector_t body_vel_cmd_;

  ros::Subscriber swing_cmd_sub_;
  BufferedValue<vector3_t> swing_cmd_buf_;
  int gaitLevel_ = 0;

  BufferedValue<feet_array_t<vector3_t>> current_feet_position_buf_;

  feet_array_t<vector3_t> latestStanceposition_;

  // std::unique_ptr<dynamic_reconfigure::Server<legged_interface::SwingTrajectoryPlannerConfig>> serverPtr_;
  feet_array_t<std::vector<bool>> eesContactFlagStocks_;
};

SwingTrajectoryPlanner::Config loadSwingTrajectorySettings(const std::string& fileName,
                                                           const std::string& fieldName = "swing_trajectory_config",
                                                           bool verbose = true);

}  // namespace legged_robot
}  // namespace ocs2
