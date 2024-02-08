
//
// Created by qiayuan on 1/24/22.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once

#include <iostream>
#include <fstream>
#include <legged_hw/LeggedHW.h>

#include <memory.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include <cstdio>
#include "Console.hpp"
#include "command.h"
#include "transmit.h"

#include <std_msgs/Float32MultiArray.h>
#include <math.h>
#include <controller_manager_msgs/SwitchController.h>

namespace legged
{
const std::vector<std::string> CONTACT_SENSOR_NAMES = { "RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT" };

struct EncosMotorData
{
  double pos_, vel_, tau_;
  double pos_des_, vel_des_, kp_, kd_, ff_;
};

struct EncosImuData
{
  double ori[4];
  double ori_cov[9];
  double angular_vel[3];
  double angular_vel_cov[9];
  double linear_acc[3];
  double linear_acc_cov[9];
};

class BridgeHW : public LeggedHW
{
public:
  BridgeHW()
  {
  }
  ros::Publisher imuAnglePublisher_;
  ros::Publisher imuyuanAnglePublisher_;
  ~BridgeHW()
  {
  }

  /** \brief Get necessary params from param server. Init hardware_interface.
   *
   * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission and
   * joint limit. Get configuration of can bus and create data pointer which point to data received from Can bus.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when init successful, False when failed.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  /** \brief Communicate with hardware. Get data, status of robot.
   *
   * Call @ref Gsmp_LEGGED_SDK::UDP::Recv() to get robot's state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void read(const ros::Time& time, const ros::Duration& period) override;

  /** \brief Comunicate with hardware. Publish command to robot.
   *
   * Propagate joint state to actuator state for the stored
   * transmission. Limit cmd_effort into suitable value. Call @ref Gsmp_LEGGED_SDK::UDP::Recv(). Publish actuator
   * current state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void write(const ros::Time& time, const ros::Duration& period) override;

private:

  bool setupJoints();

  bool setupImu();

  bool setupContactSensor(ros::NodeHandle& nh);

  EncosMotorData jointData_[10]{};
  EncosImuData imuData_{};
  int powerLimit_{};
  int contactThreshold_{};

  bool estimateContact_[4];

  YKSMotorData yksSendcmdzero_[12] = {};
  YKSMotorData yksSendcmd_[12];
  float transform_CurrentPos[12] = { 0 };

  const std::vector<int> directionMotor_{ 1, -1, 1, 1, 1, 1, -1, 1, -1, 1, 1, 1 };

  float baseMotor_[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
};

}  // namespace legged
