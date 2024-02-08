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

// Pinocchio forward declarations must be included first
#include <pinocchio/fwd.hpp>

// Pinocchio
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "legged_interface/visualization/LeggedRobotVisualizer.h"

// OCS2
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>
#include "legged_interface/gait/MotionPhaseDefinition.h"

// Additional messages not in the helpers file
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "std_msgs/Float64MultiArray.h"

// URDF related
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

namespace legged
{
using namespace ocs2;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotVisualizer::LeggedRobotVisualizer(PinocchioInterface pinocchioInterface,
                                             CentroidalModelInfo centroidalModelInfo,
                                             const PinocchioEndEffectorKinematics& endEffectorKinematics,
                                             ros::NodeHandle& nodeHandle, scalar_t maxUpdateFrequency)
  : pinocchioInterface_(std::move(pinocchioInterface))
  , centroidalModelInfo_(std::move(centroidalModelInfo))
  , endEffectorKinematicsPtr_(endEffectorKinematics.clone())
  , lastTime_(std::numeric_limits<scalar_t>::lowest())
  , minPublishTimeDifference_(1.0 / maxUpdateFrequency)
{
  endEffectorKinematicsPtr_->setPinocchioInterface(pinocchioInterface_);
  launchVisualizerNode(nodeHandle);
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::launchVisualizerNode(ros::NodeHandle& nodeHandle)
{
  costDesiredBasePositionPublisher_ =
      nodeHandle.advertise<visualization_msgs::Marker>("/legged_robot/desiredBaseTrajectory", 1);
  costDesiredFeetPositionPublishers_.resize(centroidalModelInfo_.numThreeDofContacts);
  costDesiredFeetPositionPublishers_[0] =
      nodeHandle.advertise<visualization_msgs::Marker>("/legged_robot/desiredFeetTrajectory/L", 1);
  costDesiredFeetPositionPublishers_[1] =
      nodeHandle.advertise<visualization_msgs::Marker>("/legged_robot/desiredFeetTrajectory/R", 1);
  stateOptimizedPublisher_ =
      nodeHandle.advertise<visualization_msgs::MarkerArray>("/legged_robot/optimizedStateTrajectory", 1);
  desiredTrajPublishers_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/legged_robot/desiredTrajectory", 1);
  currentStatePublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/legged_robot/currentState", 1);
  swingTrajPlotPublisher_ = nodeHandle.advertise<std_msgs::Float64MultiArray>("/data_analysis/feet_swing", 1);

  // Load URDF model
  urdf::Model urdfModel;
  if (!urdfModel.initParam("legged_robot_description"))
  {
    std::cerr << "[LeggedRobotVisualizer] Could not read URDF from: \"legged_robot_description\"" << std::endl;
  }
  else
  {
    KDL::Tree kdlTree;
    kdl_parser::treeFromUrdfModel(urdfModel, kdlTree);

    robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(kdlTree));
    robotStatePublisherPtr_->publishFixedTransforms(true);
  }
  std::string taskFile;
  nodeHandle.getParam("/taskFile", taskFile);
  loadData::loadCppDataType(taskFile, "model_settings.visualizeIK", visualize_ik_flag_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::update(const SystemObservation& observation)
{
  if (observation.time - lastTime_ > minPublishTimeDifference_)
  {
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data,
                                 centroidal_model::getGeneralizedCoordinates(observation.state, centroidalModelInfo_));
    pinocchio::updateFramePlacements(model, data);

    ros::Time timeStamp(observation.time);
    publishObservation(timeStamp, observation);
    lastTime_ = observation.time;
  }
}

void LeggedRobotVisualizer::update(const SystemObservation& observation, const PrimalSolution& primalSolution,
                                   const CommandData& command)
{
  if (observation.time - lastTime_ > minPublishTimeDifference_)
  {
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data,
                                 centroidal_model::getGeneralizedCoordinates(observation.state, centroidalModelInfo_));
    pinocchio::updateFramePlacements(model, data);

    ros::Time timeStamp(observation.time);
    publishObservation(timeStamp, observation);
    publishOptimizedStateTrajectory(timeStamp, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_,
                                    primalSolution.modeSchedule_);
    lastTime_ = observation.time;
  }
}

void LeggedRobotVisualizer::update(const SystemObservation& observation, const PrimalSolution& primalSolution,
                                   const CommandData& command,
                                   const std::shared_ptr<SwingTrajectoryPlanner>& swingTrajectoryPlanner)
{
  if (observation.time - lastTime_ > minPublishTimeDifference_)
  {
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data,
                                 centroidal_model::getGeneralizedCoordinates(observation.state, centroidalModelInfo_));
    pinocchio::updateFramePlacements(model, data);

    ros::Time timeStamp(observation.time);
    publishObservation(timeStamp, observation);
    publishDesiredTrajectory(timeStamp, command.mpcTargetTrajectories_, swingTrajectoryPlanner);
    publishOptimizedStateTrajectory(timeStamp, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_,
                                    primalSolution.modeSchedule_);
    lastTime_ = observation.time;
  }
}

void LeggedRobotVisualizer::update(const SystemObservation& observation, const CommandData& command)
{
  if (observation.time - lastTime_ > minPublishTimeDifference_)
  {
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data,
                                 centroidal_model::getGeneralizedCoordinates(observation.state, centroidalModelInfo_));
    pinocchio::updateFramePlacements(model, data);

    ros::Time timeStamp(observation.time);
    publishObservation(timeStamp, observation);
    lastTime_ = observation.time;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishObservation(const ros::Time& timeStamp, const SystemObservation& observation)
{
  // Extract components from state
  const auto basePose = centroidal_model::getBasePose(observation.state, centroidalModelInfo_);
  const auto qJoints = centroidal_model::getJointAngles(observation.state, centroidalModelInfo_);

  // Compute cartesian state and inputs
  const auto feetPositions = endEffectorKinematicsPtr_->getPosition(observation.state);
  std::vector<vector3_t> feetForces(centroidalModelInfo_.numThreeDofContacts);
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
  {
    feetForces[i] = centroidal_model::getContactForces(observation.input, i, centroidalModelInfo_);
  }

  // Publish
  publishJointTransforms(timeStamp, qJoints);
  publishBaseTransform(timeStamp, basePose);
  publishCartesianMarkers(timeStamp, modeNumber2StanceLeg(observation.mode), feetPositions, feetForces);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishReferenceTrajectory(const ros::Time& timeStamp,
                                                       const TargetTrajectories& targetTrajectories)
{
  for (int i = 0; i < targetTrajectories.size(); i++)
  {
    auto& state = targetTrajectories.stateTrajectory[i];
    // Extract components from state
    const auto basePose = centroidal_model::getBasePose(state, centroidalModelInfo_);
    const auto qJoints = centroidal_model::getJointAngles(state, centroidalModelInfo_);

    std::string tf_prefix = "next_" + std::to_string(i);
    // Publish
    publishJointTransforms(timeStamp, qJoints, tf_prefix);
    publishBaseTransform(timeStamp, basePose, tf_prefix);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishJointTransforms(const ros::Time& timeStamp, const vector_t& jointAngles,
                                                   const std::string& tf_prefix /* = ""*/) const
{
  if (robotStatePublisherPtr_ != nullptr)
  {
    std::map<std::string, scalar_t> jointPositions{
      { "leg_l1_joint", jointAngles[0] }, { "leg_l2_joint", jointAngles[1] }, { "leg_l3_joint", jointAngles[2] },
      { "leg_l4_joint", jointAngles[3] }, { "leg_l5_joint", jointAngles[4] }, { "leg_r1_joint", jointAngles[5] },
      { "leg_r2_joint", jointAngles[6] }, { "leg_r3_joint", jointAngles[7] }, { "leg_r4_joint", jointAngles[8] },
      { "leg_r5_joint", jointAngles[9] }
    };
    robotStatePublisherPtr_->publishTransforms(jointPositions, timeStamp, tf_prefix);
    robotStatePublisherPtr_->publishFixedTransforms(tf_prefix, true);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishBaseTransform(const ros::Time& timeStamp, const vector_t& basePose,
                                                 const std::string& tf_prefix /* = ""*/)
{
  if (robotStatePublisherPtr_ != nullptr)
  {
    geometry_msgs::TransformStamped baseToWorldTransform;
    baseToWorldTransform.header = getHeaderMsg(frameId_, timeStamp);
    baseToWorldTransform.child_frame_id = tf_prefix + "/base_link";

    const Eigen::Quaternion<scalar_t> q_world_base = getQuaternionFromEulerAnglesZyx(vector3_t(basePose.tail<3>()));
    baseToWorldTransform.transform.rotation = getOrientationMsg(q_world_base);
    baseToWorldTransform.transform.translation = getVectorMsg(basePose.head<3>());
    tfBroadcaster_.sendTransform(baseToWorldTransform);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishTrajectory(const std::vector<SystemObservation>& system_observation_array,
                                              scalar_t speed)
{
  for (size_t k = 0; k < system_observation_array.size() - 1; k++)
  {
    scalar_t frameDuration = speed * (system_observation_array[k + 1].time - system_observation_array[k].time);
    scalar_t publishDuration =
        timedExecutionInSeconds([&]() { publishObservation(ros::Time::now(), system_observation_array[k]); });
    if (frameDuration > publishDuration)
    {
      ros::WallDuration(frameDuration - publishDuration).sleep();
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishCartesianMarkers(const ros::Time& timeStamp, const contact_flag_t& contactFlags,
                                                    const std::vector<vector3_t>& feetPositions,
                                                    const std::vector<vector3_t>& feetForces) const
{
  // Reserve message
  const size_t numberOfCartesianMarkers = 10;
  visualization_msgs::MarkerArray markerArray;
  markerArray.markers.reserve(numberOfCartesianMarkers);

  // Feet positions and Forces
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; ++i)
  {
    markerArray.markers.emplace_back(
        getFootMarker(feetPositions[i], contactFlags[i], feetColorMap_[i], footMarkerDiameter_, footAlphaWhenLifted_));
    markerArray.markers.emplace_back(
        getForceMarker(feetForces[i], feetPositions[i], contactFlags[i], Color::green, forceScale_));
  }

  // Center of pressure
  markerArray.markers.emplace_back(getCenterOfPressureMarker(feetForces.begin(), feetForces.end(),
                                                             feetPositions.begin(), contactFlags.begin(), Color::green,
                                                             copMarkerDiameter_));

  // Support polygon
  markerArray.markers.emplace_back(getSupportPolygonMarker(
      feetPositions.begin(), feetPositions.end(), contactFlags.begin(), Color::black, supportPolygonLineWidth_));

  // Give markers an id and a frame
  assignHeader(markerArray.markers.begin(), markerArray.markers.end(), getHeaderMsg(frameId_, timeStamp));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

  // Publish cartesian markers (minus the CoM Pose)
  currentStatePublisher_.publish(markerArray);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishDesiredTrajectory(
    const ros::Time& timeStamp, const TargetTrajectories& targetTrajectories,
    const std::shared_ptr<SwingTrajectoryPlanner>& swingTrajectoryPlanner)
{
  const auto& stateTrajectory = targetTrajectories.stateTrajectory;
  const auto& inputTrajectory = targetTrajectories.inputTrajectory;
  const auto& timeTraj = targetTrajectories.timeTrajectory;

  const scalar_t time_length = timeTraj.back() - timeTraj.front();
  const scalar_t start_time = timeStamp.toSec();
  const scalar_t end_time = timeStamp.toSec() + time_length;

  size_t sample_size = floor((end_time - start_time) / 0.005);
  vector_t time_sample(sample_size);
  time_sample.setLinSpaced(sample_size, start_time, end_time);

  const auto& feet_pos_vel = swingTrajectoryPlanner->threadSaftyGetPosVel(time_sample);

  // Reserve feet swing messages
  feet_array_t<std::vector<geometry_msgs::Point>> desiredFeetSwingPositionMsgs;
  std_msgs::Float64MultiArray swing_msgs;
  swing_msgs.data.resize(7);

  for (int leg = 0; leg < 4; leg++)
  {
    geometry_msgs::Pose footPose;
    desiredFeetSwingPositionMsgs[leg].reserve(sample_size);
    for (size_t i = 0; i < sample_size; i++)
    {
      swing_msgs.data.at(0) = time_sample(i);
      swing_msgs.data.at(1) = feet_pos_vel[leg][0](i);  // pos x
      swing_msgs.data.at(2) = feet_pos_vel[leg][1](i);  // pos y
      swing_msgs.data.at(3) = feet_pos_vel[leg][2](i);  // pos z

      swing_msgs.data.at(4) = feet_pos_vel[leg][3](i);  // vel x
      swing_msgs.data.at(5) = feet_pos_vel[leg][4](i);  // vel y
      swing_msgs.data.at(6) = feet_pos_vel[leg][5](i);  // vel z

      footPose.position = getPointMsg(vector3_t(swing_msgs.data.at(1), swing_msgs.data.at(2), swing_msgs.data.at(3)));
      desiredFeetSwingPositionMsgs[leg].push_back(footPose.position);
    }
  }

  // Reserve com messages
  std::vector<geometry_msgs::Point> desiredBasePositionMsg;
  desiredBasePositionMsg.reserve(stateTrajectory.size());

  // Reserve feet messages
  feet_array_t<std::vector<geometry_msgs::Point>> desiredFeetPositionMsgs;
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
  {
    desiredFeetPositionMsgs[i].reserve(stateTrajectory.size());
  }

  for (size_t j = 0; j < stateTrajectory.size(); j++)
  {
    const auto state = stateTrajectory.at(j);
    vector_t input(centroidalModelInfo_.inputDim);
    if (j < inputTrajectory.size())
    {
      input = inputTrajectory.at(j);
    }
    else
    {
      input.setZero();
    }

    // Construct base pose msg
    const auto basePose = centroidal_model::getBasePose(state, centroidalModelInfo_);
    geometry_msgs::Pose pose;
    pose.position = getPointMsg(basePose.head<3>());

    // Fill message containers
    desiredBasePositionMsg.push_back(pose.position);

    // Fill feet msgs
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(state, centroidalModelInfo_));
    pinocchio::updateFramePlacements(model, data);

    const auto feetPositions = endEffectorKinematicsPtr_->getPosition(state);
    for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
    {
      geometry_msgs::Pose footPose;
      footPose.position = getPointMsg(feetPositions[i]);
      desiredFeetPositionMsgs[i].push_back(footPose.position);
    }
  }

  // Convert feet msgs to Array message
  visualization_msgs::MarkerArray markerArray;
  // 1 trajectory per foot + 1 for the future footholds + 1 for the com trajectory
  markerArray.markers.reserve(centroidalModelInfo_.numThreeDofContacts + 1);
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
  {
    markerArray.markers.emplace_back(
        getLineMsg(std::move(desiredFeetSwingPositionMsgs[i]), feetColorMap_[i], trajectoryLineWidth_));
    markerArray.markers.back().ns = "EE desired Trajectories";
  }
  markerArray.markers.emplace_back(getLineMsg(std::move(desiredBasePositionMsg), Color::green, trajectoryLineWidth_));
  markerArray.markers.back().ns = "CoM desired Trajectory";

  // Add headers and Id
  assignHeader(markerArray.markers.begin(), markerArray.markers.end(), getHeaderMsg(frameId_, timeStamp));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

  desiredTrajPublishers_.publish(markerArray);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishOptimizedStateTrajectory(const ros::Time& timeStamp,
                                                            const scalar_array_t& mpcTimeTrajectory,
                                                            const vector_array_t& mpcStateTrajectory,
                                                            const ModeSchedule& modeSchedule)
{
  if (mpcTimeTrajectory.empty() || mpcStateTrajectory.empty())
  {
    return;  // Nothing to publish
  }

  // Reserve Feet msg
  feet_array_t<std::vector<geometry_msgs::Point>> feetMsgs;
  std::for_each(feetMsgs.begin(), feetMsgs.end(),
                [&](std::vector<geometry_msgs::Point>& v) { v.reserve(mpcStateTrajectory.size()); });

  // Reserve Com Msg
  std::vector<geometry_msgs::Point> mpcComPositionMsgs;
  mpcComPositionMsgs.reserve(mpcStateTrajectory.size());

  // Extract Com and Feet from state
  std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const vector_t& state) {
    const auto basePose = centroidal_model::getBasePose(state, centroidalModelInfo_);

    // Fill com position and pose msgs
    geometry_msgs::Pose pose;
    pose.position = getPointMsg(basePose.head<3>());
    mpcComPositionMsgs.push_back(pose.position);

    // Fill feet msgs
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(state, centroidalModelInfo_));
    pinocchio::updateFramePlacements(model, data);

    const auto feetPositions = endEffectorKinematicsPtr_->getPosition(state);
    for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
    {
      const auto position = getPointMsg(feetPositions[i]);
      feetMsgs[i].push_back(position);
    }
  });

  // Convert feet msgs to Array message
  visualization_msgs::MarkerArray markerArray;
  markerArray.markers.reserve(centroidalModelInfo_.numThreeDofContacts +
                              2);  // 1 trajectory per foot + 1 for the future footholds + 1 for the com trajectory
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
  {
    markerArray.markers.emplace_back(getLineMsg(std::move(feetMsgs[i]), feetColorMap_[i], trajectoryLineWidth_));
    markerArray.markers.back().ns = "EE Trajectories";
  }
  markerArray.markers.emplace_back(getLineMsg(std::move(mpcComPositionMsgs), Color::red, trajectoryLineWidth_));
  markerArray.markers.back().ns = "CoM Trajectory";

  // Future footholds
  visualization_msgs::Marker sphereList;
  sphereList.type = visualization_msgs::Marker::SPHERE_LIST;
  sphereList.scale.x = footMarkerDiameter_;
  sphereList.scale.y = footMarkerDiameter_;
  sphereList.scale.z = footMarkerDiameter_;
  sphereList.ns = "Future footholds";
  sphereList.pose.orientation = getOrientationMsg({ 1., 0., 0., 0. });
  const auto& eventTimes = modeSchedule.eventTimes;
  const auto& subsystemSequence = modeSchedule.modeSequence;
  const auto tStart = mpcTimeTrajectory.front();
  const auto tEnd = mpcTimeTrajectory.back();
  for (size_t event = 0; event < eventTimes.size(); ++event)
  {
    if (tStart < eventTimes[event] && eventTimes[event] < tEnd)
    {  // Only publish future footholds within the optimized horizon
      const auto preEventContactFlags = modeNumber2StanceLeg(subsystemSequence[event]);
      const auto postEventContactFlags = modeNumber2StanceLeg(subsystemSequence[event + 1]);
      const auto postEventState =
          LinearInterpolation::interpolate(eventTimes[event], mpcTimeTrajectory, mpcStateTrajectory);

      const auto& model = pinocchioInterface_.getModel();
      auto& data = pinocchioInterface_.getData();
      pinocchio::forwardKinematics(model, data,
                                   centroidal_model::getGeneralizedCoordinates(postEventState, centroidalModelInfo_));
      pinocchio::updateFramePlacements(model, data);

      const auto feetPosition = endEffectorKinematicsPtr_->getPosition(postEventState);
      for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
      {
        if (!preEventContactFlags[i] && postEventContactFlags[i])
        {  // If a foot lands, a marker is added at that location.
          sphereList.points.emplace_back(getPointMsg(feetPosition[i]));
          sphereList.colors.push_back(getColor(feetColorMap_[i]));
        }
      }
    }
  }
  markerArray.markers.push_back(std::move(sphereList));

  // Add headers and Id
  assignHeader(markerArray.markers.begin(), markerArray.markers.end(), getHeaderMsg(frameId_, timeStamp));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

  stateOptimizedPublisher_.publish(markerArray);
}

}  // namespace legged