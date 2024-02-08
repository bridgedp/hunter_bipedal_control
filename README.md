## hunter_bipedal_control

An open source bipedal robot control framework, based on non-linear MPC and WBC, tailered for EC-hunter80-v01 bipedal robot. For more information refer to the project's [Page](https://bridgedp.github.io/hunter_bipedal_control)

## Installation

### Install dependencies

- [OCS2](https://leggedrobotics.github.io/ocs2/installation.html#prerequisites)

- [ROS1-Noetic](http://wiki.ros.org/noetic)

### Clone and Build

```shell
# Clone
mkdir -p <catkin_ws_name>/src
cd <catkin_ws_name>/src
git clone https://github.com/bridgedp/hunter_bipedal_control.git

# Build
cd <catkin_ws_name>
catkin init
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build
```

## Quick Start

### Simulation

1. Run the simulation and load the controller:

```shell
roslaunch legged_controller one_start_gazebo.launch
```

### Robot hardware

1. load the controller

```shell
roslaunch legged_controller one_start_real.launch
```

***Notes:*** After the user starts the simulation, the robot falls down in Gazebo. The user needs to press ***Ctrl+Shift+R*** to make the robot stand up. Then, dynamic parameter adjustment is used in the code, and the user needs to set ***kpposition=100***, ***kdposition=3***.

## Gamepad Control

1. Start controller

```
L1 + start
```

2. Switch to walking mode

```
Y
```

3. Use the joystick to control robot movement

The following is a schematic diagram of the handle operation:

![](./docs/f710-gallery-1.png)