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

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include "legged_interface/foot_planner/SwingTrajectoryPlanner.h"

#include <ocs2_core/misc/Lookup.h>
#include <ocs2_core/misc/Numerics.h>

#include <legged_interface/gait/MotionPhaseDefinition.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <iomanip>

namespace ocs2
{
namespace legged_robot
{
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

SwingTrajectoryPlanner::SwingTrajectoryPlanner(Config config)
  : config_(std::move(config))
  , feet_bias_{}
  , leg_swing_down_flags_{}
  , feetXTrajsBuf_{ std::move(feet_array_t<std::vector<MultiCubicSpline>>{}) }
  , feetYTrajsBuf_{ std::move(feet_array_t<std::vector<MultiCubicSpline>>{}) }
  , feetZTrajsBuf_{ std::move(feet_array_t<std::vector<MultiCubicSpline>>{}) }
  , footTrajsEventsBuf_{ std::move(std::vector<scalar_t>{}) }
  , StartStopTimeBuf_{ std::move(feet_array_t<std::vector<std::array<scalar_t, 2>>>{}) }
  , body_vel_world_buf_{ std::move(vector_t(6)) }
  , swing_cmd_buf_{ std::move(vector3_t(0, 0, 0)) }
  , current_feet_position_buf_{ std::move(feet_array_t<vector3_t>{}) }
  , latestStanceposition_{}
{
  numFeet_ = feet_bias_.size();

  feet_bias_[0] << config.feet_bias_x1, config.feet_bias_y, config.feet_bias_z;
  feet_bias_[1] << config.feet_bias_x1, -config.feet_bias_y, config.feet_bias_z;
  feet_bias_[2] << config.feet_bias_x2, config.feet_bias_y, config.feet_bias_z;
  feet_bias_[3] << config.feet_bias_x2, -config.feet_bias_y, config.feet_bias_z;

  body_vel_cmd_.resize(6);
  body_vel_cmd_.setZero();

  std::cout.setf(std::ios::fixed);
  std::cout.precision(6);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getXvelocityConstraint(size_t leg, scalar_t time) const
{
  auto index = lookup::findIndexInTimeArray(feetTrajsEvents_[leg], time);
  index = std::min((int)(feetTrajsEvents_[leg].size() - 1), index);
  return feetXTrajs_[leg][index].velocity(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getYvelocityConstraint(size_t leg, scalar_t time) const
{
  auto index = lookup::findIndexInTimeArray(feetTrajsEvents_[leg], time);
  index = std::min((int)(feetTrajsEvents_[leg].size() - 1), index);
  return feetYTrajs_[leg][index].velocity(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getZvelocityConstraint(size_t leg, scalar_t time) const
{
  auto index = lookup::findIndexInTimeArray(feetTrajsEvents_[leg], time);
  index = std::min((int)(feetTrajsEvents_[leg].size() - 1), index);
  return feetZTrajs_[leg][index].velocity(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getXpositionConstraint(size_t leg, scalar_t time) const
{
  auto index = lookup::findIndexInTimeArray(feetTrajsEvents_[leg], time);
  index = std::min((int)(feetTrajsEvents_[leg].size() - 1), index);
  auto val = feetXTrajs_[leg][index].position(time);
  return val;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getYpositionConstraint(size_t leg, scalar_t time) const
{
  auto index = lookup::findIndexInTimeArray(feetTrajsEvents_[leg], time);
  index = std::min((int)(feetTrajsEvents_[leg].size() - 1), index);
  auto val = feetYTrajs_[leg][index].position(time);
  return val;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getZpositionConstraint(size_t leg, scalar_t time) const
{
  auto index = lookup::findIndexInTimeArray(feetTrajsEvents_[leg], time);
  index = std::min((int)(feetTrajsEvents_[leg].size() - 1), index);
  auto val = feetZTrajs_[leg][index].position(time);
  return val;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::array<scalar_t, 2> SwingTrajectoryPlanner::getSwingStartStopTime(size_t leg, scalar_t time) const
{
  auto index = lookup::findIndexInTimeArray(feetTrajsEvents_[leg], time);
  index = std::min((int)(feetTrajsEvents_[leg].size() - 1), index);
  return startStopTime_[leg][index];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwingTrajectoryPlanner::update(const ModeSchedule& modeSchedule, const TargetTrajectories& targetTrajectories,
                                    scalar_t initTime)
{
  const auto& modeSequence = modeSchedule.modeSequence;
  const auto& eventTimes = modeSchedule.eventTimes;
  body_vel_world_buf_.updateFromBuffer();
  const auto& body_vel_world = body_vel_world_buf_.get();
  current_feet_position_buf_.updateFromBuffer();
  const auto& current_feet_position = current_feet_position_buf_.get();
  contact_flag_t cmd_state_leg = modeNumber2StanceLeg(modeSchedule.modeAtTime(initTime + 0.001));

  for (int i = 0; i < numFeet_; i++)
  {
    latestStanceposition_[i] = cmd_state_leg[i] ? current_feet_position[i] : latestStanceposition_[i];
    latestStanceposition_[i].z() = config_.next_position_z;
  }

  feet_array_t<vector3_t> last_stance_position = latestStanceposition_;
  feet_array_t<vector3_t> next_stance_position = latestStanceposition_;
  feet_array_t<int> last_final_idx{};

  const auto eesContactFlagStocks = extractContactFlags(modeSequence);
  eesContactFlagStocks_ = eesContactFlagStocks;

  feet_array_t<std::vector<int>> startTimesIndices;
  feet_array_t<std::vector<int>> finalTimesIndices;
  for (size_t leg = 0; leg < numFeet_; leg++)
  {
    std::tie(startTimesIndices[leg], finalTimesIndices[leg]) = updateFootSchedule(eesContactFlagStocks[leg]);
  }

  for (size_t j = 0; j < numFeet_; j++)
  {
    feetXTrajs_[j].clear();
    feetYTrajs_[j].clear();
    feetZTrajs_[j].clear();
    startStopTime_[j].clear();
    feetXTrajs_[j].reserve(modeSequence.size());
    feetYTrajs_[j].reserve(modeSequence.size());
    feetZTrajs_[j].reserve(modeSequence.size());
    startStopTime_[j].reserve(modeSequence.size());

    vector3_t last_stance_point;
    last_stance_point(0) = 0;
    last_stance_point(1) = 0;
    last_stance_point(2) = 0;
    for (int p = 0; p < modeSequence.size(); ++p)
    {
      if (!eesContactFlagStocks[j][p])
      {  // for a swing leg
        const int swingStartIndex = startTimesIndices[j][p];
        const int swingFinalIndex = finalTimesIndices[j][p];
        checkThatIndicesAreValid(j, p, swingStartIndex, swingFinalIndex, modeSequence);

        const scalar_t swingStartTime = eventTimes[swingStartIndex];
        const scalar_t swingFinalTime = eventTimes[swingFinalIndex];
        const scalar_t time_length = swingFinalTime - swingStartTime;
        startStopTime_[j].push_back({ swingStartTime, swingFinalTime });

        if (initTime < swingFinalTime && swingFinalIndex > last_final_idx[j])
        {
          last_stance_position[j] = next_stance_position[j];

          scalar_t next_middle_time = 0;
          scalar_t nextStanceFinalTime = 0;
          if (swingFinalIndex < modeSequence.size() - 1)
          {
            const int nextStanceFinalIndex = finalTimesIndices[j][swingFinalIndex + 1];
            nextStanceFinalTime = eventTimes[nextStanceFinalIndex];
            next_middle_time = (swingFinalTime + nextStanceFinalTime) / 2;
          }
          else
          {
            next_middle_time = swingFinalTime;
          }

          vector_t next_middle_body_pos = targetTrajectories.getDesiredState(next_middle_time).segment<6>(6);
          vector_t current_body_pos = targetTrajectories.getDesiredState(initTime).segment<6>(6);
          vector_t current_body_vel = targetTrajectories.stateTrajectory[0].segment<3>(0);
          next_stance_position[j] = calNextFootPos(j, initTime, swingFinalTime, next_middle_time, next_middle_body_pos,
                                                   current_body_pos, current_body_vel);
          last_final_idx[j] = swingFinalIndex;
        }

        genSwingTrajs(j, initTime, swingStartTime, swingFinalTime, last_stance_position[j], next_stance_position[j]);

      }
      else
      {
        const int stanceStartIndex = startTimesIndices[j][p];
        const int stanceFinalIndex = finalTimesIndices[j][p];

        const scalar_t stanceStartTime = eventTimes[stanceStartIndex];
        const scalar_t stanceFinalTime = eventTimes[stanceFinalIndex];
        const scalar_t time_length = stanceFinalTime - stanceStartTime;

        startStopTime_[j].push_back({ stanceStartTime, stanceFinalTime });
        const std::vector<CubicSpline::Node> x_nodes{
          CubicSpline::Node{ stanceStartTime, next_stance_position[j].x(), 0 },
          CubicSpline::Node{ stanceFinalTime, next_stance_position[j].x(), 0 }
        };
        feetXTrajs_[j].emplace_back(x_nodes);

        const std::vector<CubicSpline::Node> y_nodes{
          CubicSpline::Node{ stanceStartTime, next_stance_position[j].y(), 0 },
          CubicSpline::Node{ stanceFinalTime, next_stance_position[j].y(), 0 }
        };
        feetYTrajs_[j].emplace_back(y_nodes);
        const std::vector<CubicSpline::Node> z_nodes{
          CubicSpline::Node{ stanceStartTime, next_stance_position[j].z(), 0 },
          CubicSpline::Node{ stanceFinalTime, next_stance_position[j].z(), 0 }
        };
        feetZTrajs_[j].emplace_back(z_nodes);
      }
    }
    feetTrajsEvents_[j] = eventTimes;
  }
  feetXTrajsBuf_.setBuffer(feetXTrajs_);
  feetYTrajsBuf_.setBuffer(feetYTrajs_);
  feetZTrajsBuf_.setBuffer(feetZTrajs_);
  StartStopTimeBuf_.setBuffer(startStopTime_);
  footTrajsEventsBuf_.setBuffer(feetTrajsEvents_[0]);
}

// ref: Highly Dynamic Quadruped Locomotion via Whole-Body Impulse Control and Model Predictive Control
vector3_t SwingTrajectoryPlanner::calNextFootPos(int feet, scalar_t current_time, scalar_t stop_time,
                                                 scalar_t next_middle_time, const vector_t& next_middle_body_pos,
                                                 const vector_t& current_body_pos, const vector3_t& current_body_vel)
{
  vector3_t next_stance_position;
  vector3_t angular = next_middle_body_pos.tail(3);
  vector3_t roted_bias = getRotationMatrixFromZyxEulerAngles(angular) * feet_bias_[feet];
  vector3_t current_angular = current_body_pos.tail(3);
  auto current_rot = getRotationMatrixFromZyxEulerAngles(current_angular);
  const vector3_t& vel_cmd_linear = current_rot * body_vel_cmd_.head(3);
  const vector3_t& vel_cmd_angular = current_rot * body_vel_cmd_.tail(3);

  vector3_t current_body_vel_tmp = current_body_vel;
  current_body_vel_tmp(2) = 0;
  const vector3_t& vel_linear = current_body_vel_tmp;

  const scalar_t k = 0.03;
  vector3_t p_shoulder = (stop_time - current_time) * (0.5 * vel_linear + 0.5 * vel_cmd_linear) + roted_bias;
  vector3_t p_symmetry = (next_middle_time - stop_time) * vel_linear + k * (vel_linear - vel_cmd_linear);
  vector3_t p_centrifugal = 0.5 * sqrt(current_body_pos(2) / 9.81) * vel_linear.cross(vel_cmd_angular);
  next_stance_position = current_body_pos.head(3) + p_shoulder + p_symmetry + p_centrifugal;
  next_stance_position.z() = config_.next_position_z;
  return std::move(next_stance_position);
}

void SwingTrajectoryPlanner::genSwingTrajs(int feet, scalar_t current_time, scalar_t start_time, scalar_t stop_time,
                                           const vector3_t& start_pos, const vector3_t& stop_pos)
{
  scalar_t xy_a1 = 0.417;
  scalar_t xy_l1 = 0.650;
  scalar_t xy_k1 = 1.770;
  const std::vector<CubicSpline::Node> x_nodes{
    CubicSpline::Node{ start_time, start_pos.x(), 0 },
    CubicSpline::Node{ (1 - xy_a1) * start_time + xy_a1 * stop_time, (1 - xy_l1) * start_pos.x() + xy_l1 * stop_pos.x(),
                       xy_k1 * (stop_pos.x() - start_pos.x()) / (stop_time - start_time) },
    CubicSpline::Node{ stop_time, stop_pos.x(), 0 }
  };
  feetXTrajs_[feet].emplace_back(x_nodes);

  const std::vector<CubicSpline::Node> y_nodes{
    CubicSpline::Node{ start_time, start_pos.y(), 0 },
    CubicSpline::Node{ (1 - xy_a1) * start_time + xy_a1 * stop_time, (1 - xy_l1) * start_pos.y() + xy_l1 * stop_pos.y(),
                       xy_k1 * (stop_pos.y() - start_pos.y()) / (stop_time - start_time) },
    CubicSpline::Node{ stop_time, stop_pos.y(), 0 }
  };
  feetYTrajs_[feet].emplace_back(y_nodes);

  const scalar_t scaling = swingTrajectoryScaling(start_time, stop_time, config_.swingTimeScale);
  const scalar_t max_z = std::max(start_pos.z(), stop_pos.z()) + scaling * config_.swingHeight;
  scalar_t z_a1 = 0.251;
  scalar_t z_l1 = 0.749;
  scalar_t z_k1 = 1.338;

  scalar_t z_a2 = 0.630;
  scalar_t z_l2 = 0.570;
  scalar_t z_k2 = 1.633;

  scalar_t z_k3 = 0.000;

  const std::vector<CubicSpline::Node> z_nodes{
    CubicSpline::Node{ start_time, start_pos.z(), 0 },
    CubicSpline::Node{ (1 - z_a1) * start_time + z_a1 * stop_time, z_l1 * max_z,
                       z_k1 * (z_l1 * (max_z - start_pos.z())) / ((z_a1) * (stop_time - start_time)) },
    CubicSpline::Node{ (1 - z_a2) * start_time + z_a2 * stop_time, z_l2 * max_z + (1 - z_l2) * stop_pos.z(),
                       z_k2 * z_l2 * (stop_pos.z() - max_z) / ((1 - z_a2) * (stop_time - start_time)) },
    CubicSpline::Node{ stop_time, stop_pos.z(),
                       z_k3 * z_l2 * (stop_pos.z() - max_z) / ((1 - z_a2) * (stop_time - start_time)) }
  };
  feetZTrajs_[feet].emplace_back(z_nodes);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<std::vector<int>, std::vector<int>>
SwingTrajectoryPlanner::updateFootSchedule(const std::vector<bool>& contactFlagStock)
{
  const size_t numPhases = contactFlagStock.size();

  std::vector<int> startTimeIndexStock(numPhases, 0);
  std::vector<int> finalTimeIndexStock(numPhases, 0);

  // find the startTime and finalTime indices for swing and stance feet
  for (size_t i = 0; i < numPhases; i++)
  {
    std::tie(startTimeIndexStock[i], finalTimeIndexStock[i]) = findIndex(i, contactFlagStock);
  }
  return { startTimeIndexStock, finalTimeIndexStock };
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
feet_array_t<std::vector<bool>>
SwingTrajectoryPlanner::extractContactFlags(const std::vector<size_t>& phaseIDsStock) const
{
  const size_t numPhases = phaseIDsStock.size();

  feet_array_t<std::vector<bool>> contactFlagStock;
  std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

  for (size_t i = 0; i < numPhases; i++)
  {
    const auto contactFlag = modeNumber2StanceLeg(phaseIDsStock[i]);
    for (size_t j = 0; j < numFeet_; j++)
    {
      contactFlagStock[j][i] = contactFlag[j];
    }
  }
  return contactFlagStock;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<int, int> SwingTrajectoryPlanner::findIndex(size_t index, const std::vector<bool>& contactFlagStock)
{
  const size_t numPhases = contactFlagStock.size();

  // find the starting time
  int startTimesIndex = 0;
  for (int ip = index - 1; ip >= 0; ip--)
  {
    if (contactFlagStock[ip] != contactFlagStock[index])
    {
      startTimesIndex = ip;
      break;
    }
  }

  // find the final time
  int finalTimesIndex = numPhases - 2;
  for (size_t ip = index + 1; ip < numPhases; ip++)
  {
    if (contactFlagStock[ip] != contactFlagStock[index])
    {
      finalTimesIndex = ip - 1;
      break;
    }
  }

  return { startTimesIndex, finalTimesIndex };
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwingTrajectoryPlanner::checkThatIndicesAreValid(int leg, int index, int startIndex, int finalIndex,
                                                      const std::vector<size_t>& phaseIDsStock)
{
  const size_t numSubsystems = phaseIDsStock.size();
  if (startIndex < 0)
  {
    std::cerr << "Subsystem: " << index << " out of " << numSubsystems - 1 << std::endl;
    for (size_t i = 0; i < numSubsystems; i++)
    {
      std::cerr << "[" << i << "]: " << phaseIDsStock[i] << ",  ";
    }
    std::cerr << std::endl;

    throw std::runtime_error("The time of take-off for the first swing of the EE with ID " + std::to_string(leg) +
                             " is not defined.");
  }
  if (finalIndex >= numSubsystems - 1)
  {
    std::cerr << "Subsystem: " << index << " out of " << numSubsystems - 1 << std::endl;
    for (size_t i = 0; i < numSubsystems; i++)
    {
      std::cerr << "[" << i << "]: " << phaseIDsStock[i] << ",  ";
    }
    std::cerr << std::endl;

    throw std::runtime_error("The time of touch-down for the last swing of the EE with ID " + std::to_string(leg) +
                             " is not defined.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
feet_array_t<std::array<vector_t, 6>> SwingTrajectoryPlanner::threadSaftyGetPosVel(const vector_t& time_sample)
{
  feetXTrajsBuf_.updateFromBuffer();
  feetYTrajsBuf_.updateFromBuffer();
  feetZTrajsBuf_.updateFromBuffer();
  footTrajsEventsBuf_.updateFromBuffer();
  const auto& Xs = feetXTrajsBuf_.get();
  const auto& Ys = feetYTrajsBuf_.get();
  const auto& Zs = feetZTrajsBuf_.get();
  const auto& footTrajsEvents_ = footTrajsEventsBuf_.get();

  size_t sample_size = time_sample.size();

  feet_array_t<std::array<vector_t, 6>> feet_pos_vel;
  for (int leg = 0; leg < numFeet_; leg++)
  {
    for (int j = 0; j < feet_pos_vel[leg].size(); j++)
    {
      feet_pos_vel[leg][j].resize(sample_size);
    }
  }

  for (size_t i = 0; i < sample_size; i++)
  {
    auto index = lookup::findIndexInTimeArray(footTrajsEvents_, time_sample(i));
    index = std::min((int)(footTrajsEvents_.size() - 1), index);

    for (int leg = 0; leg < numFeet_; leg++)
    {
      feet_pos_vel[leg][0][i] = Xs[leg][index].position(time_sample(i));
      feet_pos_vel[leg][1][i] = Ys[leg][index].position(time_sample(i));
      feet_pos_vel[leg][2][i] = Zs[leg][index].position(time_sample(i));

      feet_pos_vel[leg][3][i] = Xs[leg][index].velocity(time_sample(i));
      feet_pos_vel[leg][4][i] = Ys[leg][index].velocity(time_sample(i));
      feet_pos_vel[leg][5][i] = Zs[leg][index].velocity(time_sample(i));
    }
  }
  return std::move(feet_pos_vel);
}

feet_array_t<std::array<scalar_t, 2>> SwingTrajectoryPlanner::threadSaftyGetStartStopTime(scalar_t time)
{
  feet_array_t<std::array<scalar_t, 2>> startStopTime4legs;
  StartStopTimeBuf_.updateFromBuffer();
  footTrajsEventsBuf_.updateFromBuffer();
  const auto& Ts = StartStopTimeBuf_.get();
  const auto& footTrajsEvents_ = footTrajsEventsBuf_.get();
  auto index = lookup::findIndexInTimeArray(footTrajsEvents_, time);
  index = std::min((int)(footTrajsEvents_.size() - 1), index);
  for (int leg = 0; leg < numFeet_; leg++)
  {
    if (Ts[leg].size())
    {
      startStopTime4legs[leg] = Ts[leg][index];
    }
    else
    {
      startStopTime4legs[leg] = std::array<scalar_t, 2>{ time, time };
    }
  }

  return std::move(startStopTime4legs);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwingTrajectoryPlanner::Config loadSwingTrajectorySettings(const std::string& fileName, const std::string& fieldName,
                                                           bool verbose)
{
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(fileName, pt);

  if (verbose)
  {
    std::cerr << "\n #### Swing Trajectory Config:";
    std::cerr << "\n #### =============================================================================\n";
  }

  SwingTrajectoryPlanner::Config config;
  const std::string prefix = fieldName + ".";

  loadData::loadPtreeValue(pt, config.liftOffVelocity, prefix + "liftOffVelocity", verbose);
  loadData::loadPtreeValue(pt, config.touchDownVelocity, prefix + "touchDownVelocity", verbose);
  loadData::loadPtreeValue(pt, config.swingHeight, prefix + "swingHeight", verbose);
  loadData::loadPtreeValue(pt, config.swingTimeScale, prefix + "swingTimeScale", verbose);
  loadData::loadPtreeValue(pt, config.feet_bias_x1, prefix + "feet_bias_x1", verbose);
  loadData::loadPtreeValue(pt, config.feet_bias_x2, prefix + "feet_bias_x2", verbose);
  loadData::loadPtreeValue(pt, config.feet_bias_y, prefix + "feet_bias_y", verbose);
  loadData::loadPtreeValue(pt, config.feet_bias_z, prefix + "feet_bias_z", verbose);
  loadData::loadPtreeValue(pt, config.next_position_z, prefix + "next_position_z", verbose);

  if (verbose)
  {
    std::cerr << " #### =============================================================================" << std::endl;
  }

  return config;
}

}  // namespace legged_robot
}  // namespace ocs2
