
//
// Created by qiayuan on 1/24/22.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include "legged_hw/LeggedHW.h"

namespace legged
{
bool LeggedHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& /*robot_hw_nh*/)
{
  if (!loadUrdf(root_nh))
  {
    ROS_ERROR("Error occurred while setting up urdf");
    return false;
  }

  registerInterface(&jointStateInterface_);
  registerInterface(&hybridJointInterface_);
  registerInterface(&imuSensorInterface_);
  registerInterface(&contactSensorInterface_);

  return true;
}

bool LeggedHW::loadUrdf(ros::NodeHandle& rootNh)
{
  std::string urdfString;
  if (urdfModel_ == nullptr)
  {
    urdfModel_ = std::make_shared<urdf::Model>();
  }
  
  rootNh.getParam("legged_robot_description", urdfString);
  return !urdfString.empty() && urdfModel_->initString(urdfString);
}

}  // namespace legged
