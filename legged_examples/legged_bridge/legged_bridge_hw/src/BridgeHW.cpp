#include "legged_bridge_hw/BridgeHW.h"
#include "std_msgs/Float64MultiArray.h"
#include <ostream>
#include <vector>
#include <iostream>
#include <cmath>

namespace legged
{
bool BridgeHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  root_nh.setParam("gsmp_controller_switch", "null");
  int ec_slavecount = EtherCAT_Init("enp45s0");
  std::cout << "开始EtherCAT初始化" << std::endl;
  if (ec_slavecount <= 0)
  {
    std::cout << "未找到从站，程序退出" << std::endl;
    return false;
  }

  EtherCAT_Send_Command((YKSMotorData*)yksSendcmdzero_);

  EtherCAT_Get_State();

  if (!LeggedHW::init(root_nh, robot_hw_nh))
    return false;

  robot_hw_nh.getParam("power_limit", powerLimit_);

  setupJoints();
  setupImu();
  return true;
}

void BridgeHW::read(const ros::Time& time, const ros::Duration& period)
{
  EtherCAT_Get_State();
  for (int i = 0; i < 12; i++)
  {
    jointData_[i].pos_ = (motorDate_recv[i].pos_ - baseMotor_[i]) * directionMotor_[i];
    jointData_[i].vel_ = motorDate_recv[i].vel_ * directionMotor_[i];
    jointData_[i].tau_ = motorDate_recv[i].tau_ * directionMotor_[i];
  }

  imuData_.ori[0] = imuData_recv.quat_float[2];          
  imuData_.ori[1] = -imuData_recv.quat_float[1];
  imuData_.ori[2] = imuData_recv.quat_float[3];
  imuData_.ori[3] = imuData_recv.quat_float[0];
  imuData_.angular_vel[0] = imuData_recv.gyro_float[1];  
  imuData_.angular_vel[1] = -imuData_recv.gyro_float[0];
  imuData_.angular_vel[2] = imuData_recv.gyro_float[2];
  imuData_.linear_acc[0] = imuData_recv.accel_float[1];   
  imuData_.linear_acc[1] = -imuData_recv.accel_float[0];
  imuData_.linear_acc[2] = imuData_recv.accel_float[2];

  std::vector<std::string> names = hybridJointInterface_.getNames();
  for (const auto& name : names)
  {
    HybridJointHandle handle = hybridJointInterface_.getHandle(name);
    handle.setFeedforward(0.);
    handle.setVelocityDesired(0.);
    handle.setKd(3.0);
    handle.setKp(0.);
  }
}

void BridgeHW::write(const ros::Time& time, const ros::Duration& period)
{
  for (int i = 0; i < 12; ++i)
  {
    yksSendcmd_[i].pos_des_ = jointData_[i].pos_des_ * directionMotor_[i] + baseMotor_[i];
    yksSendcmd_[i].vel_des_ = jointData_[i].vel_des_ * directionMotor_[i];

    if (i == 0 || i == 1 || i == 5 || i == 6)
    {
      yksSendcmd_[i].kp_ = 0.7 * jointData_[i].kp_;
      yksSendcmd_[i].kd_ = 0.7 * jointData_[i].kd_;
      yksSendcmd_[i].ff_ = 0.7 * jointData_[i].ff_ * directionMotor_[i];
    }
    else
    {
      yksSendcmd_[i].kp_ = jointData_[i].kp_;
      yksSendcmd_[i].kd_ = jointData_[i].kd_;
      yksSendcmd_[i].ff_ = jointData_[i].ff_ * directionMotor_[i];
    }
  }
  EtherCAT_Send_Command((YKSMotorData*)yksSendcmd_);
}

bool BridgeHW::setupJoints()
{
  for (const auto& joint : urdfModel_->joints_)
  {
    int leg_index, joint_index;
    if (joint.first.find("leg_l") != std::string::npos)
    {
      leg_index = 0;
    }
    else if (joint.first.find("leg_r") != std::string::npos)
    {
      leg_index = 1;
    }
    else
      continue;
    if (joint.first.find("1_joint") != std::string::npos)
      joint_index = 0;
    else if (joint.first.find("2_joint") != std::string::npos)
      joint_index = 1;
    else if (joint.first.find("3_joint") != std::string::npos)
      joint_index = 2;
    else if (joint.first.find("4_joint") != std::string::npos)
      joint_index = 3;
    else if (joint.first.find("5_joint") != std::string::npos)
      joint_index = 4;
    else
      continue;

    int index = leg_index * 5 + joint_index;
    hardware_interface::JointStateHandle state_handle(joint.first, &jointData_[index].pos_, &jointData_[index].vel_,
                                                      &jointData_[index].tau_);
    jointStateInterface_.registerHandle(state_handle);
    hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &jointData_[index].pos_des_,
                                                           &jointData_[index].vel_des_, &jointData_[index].kp_,
                                                           &jointData_[index].kd_, &jointData_[index].ff_));
  }
  return true;
}

bool BridgeHW::setupImu()
{
  imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle(
      "imu_link", "imu_link", imuData_.ori, imuData_.ori_cov, imuData_.angular_vel, imuData_.angular_vel_cov,
      imuData_.linear_acc, imuData_.linear_acc_cov));
  imuData_.ori_cov[0] = 0.0012;
  imuData_.ori_cov[4] = 0.0012;
  imuData_.ori_cov[8] = 0.0012;

  imuData_.angular_vel_cov[0] = 0.0004;
  imuData_.angular_vel_cov[4] = 0.0004;
  imuData_.angular_vel_cov[8] = 0.0004;

  return true;
}

}  // namespace legged
