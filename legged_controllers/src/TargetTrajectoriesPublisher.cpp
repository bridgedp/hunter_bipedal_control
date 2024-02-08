//
// Created by qiayuan on 2022/7/24.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include "legged_controllers/TargetTrajectoriesPublisher.h"

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

using namespace legged;

namespace
{
scalar_t TARGET_DISPLACEMENT_VELOCITY;
scalar_t TARGET_ROTATION_VELOCITY;
scalar_t COM_HEIGHT;
vector_t DEFAULT_JOINT_STATE(10);
scalar_t TIME_TO_TARGET;
}  // namespace

scalar_t estimateTimeToTarget(const vector_t& desiredBaseDisplacement)
{
  const scalar_t& dx = desiredBaseDisplacement(0);
  const scalar_t& dy = desiredBaseDisplacement(1);
  const scalar_t& dyaw = desiredBaseDisplacement(3);
  const scalar_t rotationTime = std::abs(dyaw) / TARGET_ROTATION_VELOCITY;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacementTime = displacement / TARGET_DISPLACEMENT_VELOCITY;
  return std::max(rotationTime, displacementTime);
}

TargetTrajectories targetPoseToTargetTrajectories(const vector_t& targetPose, const SystemObservation& observation,
                                                  const scalar_t& targetReachingTime)
{
  // desired time trajectory
  const scalar_array_t timeTrajectory{ observation.time, targetReachingTime };

  // desired state trajectory
  vector_t currentPose = observation.state.segment<6>(6);
  scalar_t dz = COM_HEIGHT - currentPose(2);
  dz = dz > 0 ? fmin(dz, changeLimit_[2]) : fmax(dz, -changeLimit_[2]);
  currentPose(2) = currentPose(2) + dz;
  currentPose(4) = 0;
  currentPose(5) = 0;
  vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
  stateTrajectory[0] << vector_t::Zero(6), currentPose, DEFAULT_JOINT_STATE;
  stateTrajectory[1] << vector_t::Zero(6), targetPose, DEFAULT_JOINT_STATE;

  const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));
  return { timeTrajectory, stateTrajectory, inputTrajectory };
}

TargetTrajectories bodyRotationToTargetTrajectories(const vector_t& targetPose, const SystemObservation& observation,
                                                    const scalar_t& targetReachingTime)
{
  // desired time trajectory
  const scalar_array_t timeTrajectory{ observation.time, targetReachingTime };

  // desired state trajectory
  vector_t currentPose = observation.state.segment<6>(6);
  scalar_t dz = COM_HEIGHT - currentPose(2);
  dz = dz > 0 ? fmin(dz, changeLimit_[2]) : fmax(dz, -changeLimit_[2]);
  currentPose(2) = currentPose(2) + dz;
  currentPose(3) = targetPose(3);
  currentPose(4) = targetPose(4);
  currentPose(5) = targetPose(5);
  vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
  stateTrajectory[0] << vector_t::Zero(6), currentPose, DEFAULT_JOINT_STATE;
  stateTrajectory[1] << vector_t::Zero(6), targetPose, DEFAULT_JOINT_STATE;

  const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));
  return { timeTrajectory, stateTrajectory, inputTrajectory };
}

TargetTrajectories goalToTargetTrajectories(const vector_t& goal, const SystemObservation& observation)
{
  const vector_t currentPose = observation.state.segment<6>(6);
  const vector_t targetPose = [&]() {
    vector_t target(6);
    target(0) = goal(0);
    target(1) = goal(1);
    scalar_t dz = COM_HEIGHT - currentPose(2);
    dz = dz > 0 ? fmin(dz, changeLimit_[2]) : fmax(dz, -changeLimit_[2]);
    target(2) = currentPose(2) + dz;
    target(3) = goal(3);
    target(4) = 0;
    target(5) = 0;
    return target;
  }();
  const scalar_t targetReachingTime = observation.time + estimateTimeToTarget(targetPose - currentPose);
  return targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
}

TargetTrajectories cmdVelToTargetTrajectories(const vector_t& cmdVel, const SystemObservation& observation)
{
  const vector_t currentPose = observation.state.segment<6>(6);
  const Eigen::Matrix<scalar_t, 3, 1> zyx = currentPose.tail(3);
  vector_t cmdVelRot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdVel.head(3);
  const scalar_t timeToTarget = TIME_TO_TARGET;
  scalar_t z_change = cmdVelRot(3) * timeToTarget;
  if (fabs(cmdVelRot(0)) < 0.06)
    cmdVelRot(0) = 0;
  else if (fabs(cmdVelRot(1)) < 0.06)
    cmdVelRot(1) = 0;
  const vector_t targetPose = [&]() {
    vector_t target(6);
    target(0) = currentPose(0) + cmdVelRot(0) * timeToTarget;
    target(1) = currentPose(1) + cmdVelRot(1) * timeToTarget;
    target(2) = COM_HEIGHT;
    target(3) = currentPose(3) + cmdVel(3) * timeToTarget;
    target(4) = 0;
    target(5) = 0;
    return target;
  }();

  // target reaching duration
  const scalar_t targetReachingTime = observation.time + timeToTarget;
  auto trajectories = targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
  trajectories.stateTrajectory[0].head(3) = cmdVelRot;
  trajectories.stateTrajectory[1].head(3) = cmdVelRot;
  return trajectories;
}

TargetTrajectories cmdPosToTargetTrajectories(const vector_t& cmdPos, const SystemObservation& observation)
{
  const vector_t currentPose = observation.state.segment<6>(6);
  const scalar_t timeToTarget = TIME_TO_TARGET;
  const vector_t targetPose = [&]() {
    vector_t target(6);
    target(0) = currentPose(0);
    target(1) = currentPose(1);
    scalar_t dz = COM_HEIGHT - currentPose(2);
    dz = dz > 0 ? fmin(dz, changeLimit_[2]) : fmax(dz, -changeLimit_[2]);
    target(2) = currentPose(2) + dz;
    target(3) = cmdPos[0];
    target(4) = cmdPos[1];
    target(5) = cmdPos[2];
    return target;
  }();
  const scalar_t targetReachingTime = observation.time + timeToTarget;
  auto trajectories = bodyRotationToTargetTrajectories(targetPose, observation, targetReachingTime);
  return trajectories;
}

int main(int argc, char** argv)
{
  const std::string robotName = "legged_robot";

  // Initialize ros node
  ::ros::init(argc, argv, robotName + "_target");
  ::ros::NodeHandle nodeHandle;
  // Get node parameters
  std::string referenceFile;
  std::string taskFile;
  nodeHandle.getParam("/referenceFile", referenceFile);
  nodeHandle.getParam("/taskFile", taskFile);

  loadData::loadCppDataType(referenceFile, "comHeight", COM_HEIGHT);
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", DEFAULT_JOINT_STATE);
  loadData::loadCppDataType(referenceFile, "targetRotationVelocity", TARGET_ROTATION_VELOCITY);
  loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", TARGET_DISPLACEMENT_VELOCITY);
  loadData::loadCppDataType(taskFile, "mpc.timeHorizon", TIME_TO_TARGET);

  TargetTrajectoriesPublisher target_pose_command(nodeHandle, robotName, &goalToTargetTrajectories,
                                                  &cmdVelToTargetTrajectories, &cmdPosToTargetTrajectories);

  ros::spin();
  return 0;
}
