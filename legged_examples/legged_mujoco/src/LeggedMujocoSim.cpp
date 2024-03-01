

#include "legged_mujoco/LeggedMujocoSim.h"


namespace legged {

bool MujocoSIM::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  if (!LeggedHW::init(root_nh, robot_hw_nh)) {
    return false;
  }

  setupJoints();
  setupImu();
  // setupContactSensor(robot_hw_nh);

  mujocoLcm_ = std::make_shared<MujocoLcm>();

  mujocoLcm_->startLCMThread();

  // f = boost::bind(&MujocoSIM::ConfigCallback, this, _1);
  // server.setCallback(f);
  // udp_->InitCmdData(lowCmd_);

  return true;
}    


void MujocoSIM::read(const ros::Time& time, const ros::Duration& /*period*/) {
  mujocoLcm_->GetRecv(lowState_);
  for (int i = 0; i < 10; ++i) {
    jointData_[i].pos_ = lowState_.joint_pos[i];
    jointData_[i].vel_ = lowState_.joint_vel[i];
    jointData_[i].tau_ = lowState_.joint_torque[i];
  }

  imuData_.ori_[0] = lowState_.quaternion[1];
  imuData_.ori_[1] = lowState_.quaternion[2];
  imuData_.ori_[2] = lowState_.quaternion[3];
  imuData_.ori_[3] = lowState_.quaternion[0];
  imuData_.angularVel_[0] = lowState_.gyroscope[0];
  imuData_.angularVel_[1] = lowState_.gyroscope[1];
  imuData_.angularVel_[2] = lowState_.gyroscope[2];
  imuData_.linearAcc_[0] = lowState_.accelerometer[0];
  imuData_.linearAcc_[1] = lowState_.accelerometer[1];
  imuData_.linearAcc_[2] = lowState_.accelerometer[2];

  // for (int i = 0; i < 4; ++i) {
  //   contactState_[i] = lowState_.foot_force[i] > contactThreshold_;
  // }

}

void MujocoSIM::write(const ros::Time& time, const ros::Duration& /*period*/) {

  lowCmd_.timestamp = time.toNSec();
  for (int i = 0; i < 10; ++i) {
    lowCmd_.joint_pos[i] = static_cast<double>(jointData_[i].posDes_);
    lowCmd_.joint_vel[i] = static_cast<double>(jointData_[i].velDes_);
    lowCmd_.kp[i] = static_cast<double>(jointData_[i].kp_);
    lowCmd_.kd[i] = static_cast<double>(jointData_[i].kd_);
    lowCmd_.ff_tau[i] = static_cast<double>(jointData_[i].ff_);
  }

  mujocoLcm_->SetSend(lowCmd_);
  mujocoLcm_->Send();
}


bool MujocoSIM::setupJoints() {
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
    hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &jointData_[index].posDes_,
                                                           &jointData_[index].velDes_, &jointData_[index].kp_,
                                                           &jointData_[index].kd_, &jointData_[index].ff_));
  }
  return true;
}

bool MujocoSIM::setupImu() {
  imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle("imu_link", "imu_link", imuData_.ori_, imuData_.oriCov_,
                                                                         imuData_.angularVel_, imuData_.angularVelCov_, imuData_.linearAcc_,
                                                                         imuData_.linearAccCov_));
  imuData_.oriCov_[0] = 0.0012;
  imuData_.oriCov_[4] = 0.0012;
  imuData_.oriCov_[8] = 0.0012;

  imuData_.angularVelCov_[0] = 0.0004;
  imuData_.angularVelCov_[4] = 0.0004;
  imuData_.angularVelCov_[8] = 0.0004;

  return true;
}

// bool MujocoSIM::setupContactSensor(ros::NodeHandle& nh) {
//   nh.getParam("contact_threshold", contactThreshold_);
//   for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
//     contactSensorInterface_.registerHandle(ContactSensorHandle(CONTACT_SENSOR_NAMES[i], &contactState_[i]));
//   }
//   return true;
// }


MujocoSIM:: ~MujocoSIM(){
  mujocoLcm_->joinLCMThread();
}

}

