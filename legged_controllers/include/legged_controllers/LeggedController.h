//
// Created by qiayuan on 2022/6/24.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <legged_interface/visualization/LeggedRobotVisualizer.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <legged_estimation/StateEstimateBase.h>
#include <legged_interface/LeggedInterface.h>
#include <legged_wbc/WbcBase.h>
#include "legged_controllers/SafetyChecker.h"
#include "legged_controllers/visualization/LeggedSelfCollisionVisualization.h"
#include "std_msgs/Float64MultiArray.h"
#include <atomic>
#include "std_msgs/Float32.h"
#include <dynamic_reconfigure/server.h>
#include "legged_controllers/TutorialsConfig.h"
#include <ocs2_robotic_tools/common/LoopshapingRobotInterface.h>
#include "legged_interface/foot_planner/InverseKinematics.h"

namespace legged
{
using namespace ocs2;
using namespace legged_robot;

class LeggedController
  : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                          ContactSensorInterface>
{
public:
  LeggedController() = default;
  ~LeggedController() override;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& /*time*/) override
  {
    mpcRunning_ = false;
  }

protected:
  virtual void updateStateEstimation(const ros::Time& time, const ros::Duration& period);

  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile,
                                    const std::string& referenceFile, bool verbose);
  virtual void setupMpc();
  virtual void setupMrt();
  virtual void setupStateEstimate(const std::string& taskFile, bool verbose);

  void dynamicParamCallback(legged_controllers::TutorialsConfig& config, uint32_t level);

  void setWalkCallback(const std_msgs::Float32::ConstPtr& msg);
  void loadControllerCallback(const std_msgs::Float32::ConstPtr& msg);
  void EmergencyStopCallback(const std_msgs::Float32::ConstPtr& msg);
  void ResetTargetCallback(const std_msgs::Float32::ConstPtr& msg);

  void resetMPC();
  void RetrievingParameters();
  void ModeSubscribe();

  // Interface
  std::shared_ptr<LeggedInterface> leggedInterface_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;
  std::vector<HybridJointHandle> hybridJointHandles_;
  std::vector<ContactSensorHandle> contactHandles_;
  hardware_interface::ImuSensorHandle imuSensorHandle_;

  // State Estimation
  SystemObservation currentObservation_;
  vector_t measuredRbdState_;
  std::shared_ptr<StateEstimateBase> stateEstimate_;
  std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;

  // Whole Body Control
  std::shared_ptr<WbcBase> wbc_;
  std::shared_ptr<SafetyChecker> safetyChecker_;

  // Nonlinear MPC
  std::shared_ptr<MPC_BASE> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

  // Visualization
  std::shared_ptr<legged::LeggedRobotVisualizer> robotVisualizer_;
  std::shared_ptr<LeggedSelfCollisionVisualization> selfCollisionVisualization_;
  ros::Publisher observationPublisher_;

  // Emergency stop
  ros::Subscriber subSetWalk_;
  ros::Subscriber subLoadcontroller_;
  ros::Subscriber subEmgstop_;
  ros::Subscriber subResetTarget_;

  ros::Duration startingTime_;

  std::unique_ptr<dynamic_reconfigure::Server<legged_controllers::TutorialsConfig>> serverPtr_;

private:
  std::thread mpcThread_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};
  benchmark::RepeatedTimer mpcTimer_;
  benchmark::RepeatedTimer wbcTimer_;

  bool loadControllerFlag_{ false };
  bool setWalkFlag_{ false };
  bool emergencyStopFlag_{ false };
  std::atomic_bool firstStartMpc_{ false };

  // Initializing based on the number of joints
  vector_t posDes_ = vector_t::Zero(10);
  vector_t velDes_ = vector_t::Zero(10);
  vector_t posDesOutput_ = vector_t::Zero(10);
  vector_t velDesOutput_ = vector_t::Zero(10);

  size_t stateDim_{ 0 };
  size_t inputDim_{ 0 };
  size_t jointDim_{ 0 };
  size_t footDim_{ 0 };
  size_t gencoordDim_{ 0 };
  size_t dofPerLeg_{ 0 };

  std::atomic<scalar_t> kp_position{ 0 };
  std::atomic<scalar_t> kd_position{ 0 };
  std::atomic<scalar_t> kp_big_stance{ 0 };
  std::atomic<scalar_t> kp_big_swing{ 0 };
  std::atomic<scalar_t> kp_small_stance{ 0 };
  std::atomic<scalar_t> kp_small_swing{ 0 };
  std::atomic<scalar_t> kd_small{ 0 };
  std::atomic<scalar_t> kd_big{ 0 };
  std::atomic<scalar_t> kp_feet_stance{ 0 };
  std::atomic<scalar_t> kp_feet_swing{ 0 };
  std::atomic<scalar_t> kd_feet{ 0 };

  vector_t defalutJointPos_;

  InverseKinematics inverseKinematics_;
};

}  // namespace legged