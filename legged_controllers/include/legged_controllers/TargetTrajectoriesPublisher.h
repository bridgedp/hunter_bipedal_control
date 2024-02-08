//
// Created by qiayuan on 2022/7/24.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <ros/subscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include "std_msgs/Float32.h"
#include <atomic>

namespace legged
{
using namespace ocs2;

vector_t lastVel_(4);
vector_t changeLimit_(4);

class TargetTrajectoriesPublisher final
{
public:
  using CmdToTargetTrajectories =
      std::function<TargetTrajectories(const vector_t& cmd, const SystemObservation& observation)>;

  TargetTrajectoriesPublisher(::ros::NodeHandle& nh, const std::string& topicPrefix,
                              CmdToTargetTrajectories goalToTargetTrajectories,
                              CmdToTargetTrajectories cmdVelToTargetTrajectories,
                              CmdToTargetTrajectories cmdPosToTargetTrajectories)
    : goalToTargetTrajectories_(std::move(goalToTargetTrajectories))
    , cmdVelToTargetTrajectories_(std::move(cmdVelToTargetTrajectories))
    , cmdPosToTargetTrajectories_(std::move(cmdPosToTargetTrajectories))
    , tf2_(buffer_)
  {
    // Trajectories publisher
    targetTrajectoriesPublisher_.reset(new TargetTrajectoriesRosPublisher(nh, topicPrefix));
    cmdVelfilteredpub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel_filtered", 1);
    gaitTypepub_ = nh.advertise<std_msgs::Int32>("/gait_type", 1);

    // observation subscriber
    auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);

      deadTimeforswitch_ = deadTimeforswitch_ > 0 ? (deadTimeforswitch_ - 1) : 0;
    };
    observationSub_ =
        nh.subscribe<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 1, observationCallback);

    // goal subscriber
    auto goalCallback = [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      if (latestObservation_.time == 0.0)
      {
        return;
      }
      geometry_msgs::PoseStamped pose = *msg;
      try
      {
        buffer_.transform(pose, pose, "odom", ros::Duration(0.2));
      }
      catch (tf2::TransformException& ex)
      {
        ROS_WARN("Failure %s\n", ex.what());
        return;
      }

      vector_t cmdGoal = vector_t::Zero(6);
      cmdGoal[0] = pose.pose.position.x;
      cmdGoal[1] = pose.pose.position.y;
      cmdGoal[2] = pose.pose.position.z;
      Eigen::Quaternion<scalar_t> q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y,
                                    pose.pose.orientation.z);
      cmdGoal[3] = q.toRotationMatrix().eulerAngles(0, 1, 2).z();
      cmdGoal[4] = q.toRotationMatrix().eulerAngles(0, 1, 2).y();
      cmdGoal[5] = q.toRotationMatrix().eulerAngles(0, 1, 2).x();

      const auto trajectories = goalToTargetTrajectories_(cmdGoal, latestObservation_);
      targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    };

    changeLimit_ << 0.1, 0.05, 0.04, 0.3;
    lastVel_ << 0, 0, 0, 0;

    // cmd_vel subscriber
    auto cmdVelCallback = [this](const geometry_msgs::Twist::ConstPtr& msg) {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      if (latestObservation_.time == 0.0 || !velCmdconnection_)
      {
        return;
      }
      auto dx = msg->linear.x - lastVel_[0];
      dx = dx > 0 ? fmin(dx, changeLimit_[0]) : fmax(dx, -changeLimit_[0]);
      lastVel_[0] += dx;

      auto dy = msg->linear.y - lastVel_[1];
      dy = dy > 0 ? fmin(dy, changeLimit_[1]) : fmax(dy, -changeLimit_[1]);
      lastVel_[1] += dy;

      lastVel_[2] = 0;

      auto dyaw = msg->angular.z - lastVel_[3];
      dyaw = dyaw > 0 ? fmin(dyaw, changeLimit_[3]) : fmax(dyaw, -changeLimit_[3]);
      lastVel_[3] += dyaw;

      geometry_msgs::Twist filter_msg = *msg;
      filter_msg.linear.x = lastVel_[0];
      filter_msg.linear.y = lastVel_[1];
      filter_msg.linear.z = lastVel_[2];
      filter_msg.angular.z = lastVel_[3];

      cmdVelfilteredpub_.publish(filter_msg);

      const auto trajectories = cmdVelToTargetTrajectories_(lastVel_, latestObservation_);
      targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    };

    goalSub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, goalCallback);
    cmdVelSub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmdVelCallback);

    auto stance_switch_callback = [this](const std_msgs::Float32::ConstPtr& msg) {
      if (deadTimeforswitch_ <= 0)
      {
        if (!isRunning_)
        {
          isAutostance_ = !isAutostance_;
          std_msgs::Int32 gaitType_;
          gaitType_.data = isAutostance_ ? 0 : 2;
          gaitTypepub_.publish(gaitType_);
        }
      }
      deadTimeforswitch_ = 100;
    };

    stanceSwitchsub_ = nh.subscribe<std_msgs::Float32>("/stance_switch", 1, stance_switch_callback);
  }

private:
  CmdToTargetTrajectories goalToTargetTrajectories_, cmdVelToTargetTrajectories_, cmdPosToTargetTrajectories_;

  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisher_;

  ::ros::Subscriber observationSub_, goalSub_, cmdVelSub_, comAdjustmentSub_, runningSwitchsub_, stanceSwitchsub_,
      bodyRotationsub_, bodyrotationSwitchsub_, pawupSwitchsub_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;

  mutable std::mutex latestObservationMutex_;
  SystemObservation latestObservation_;

  ros::Publisher cmdVelfilteredpub_;
  ros::Publisher gaitTypepub_;

  std::atomic_bool isRunning_{};
  std::atomic_bool isPawup_{};
  std::atomic_bool isAutostance_{};
  std::atomic_int32_t deadTimeforswitch_{ 0 };
  std::atomic_bool velCmdconnection_{ true };

};

}  // namespace legged
