# kuka_controllers
ROS2 controllers necessary for KUKA robots

Github CI | SonarCloud
------------| ---------------
[![Build Status](https://github.com/kroshu/kuka_controllers/workflows/CI/badge.svg?branch=main)](https://github.com/kroshu/kuka_controllers/actions) | [![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=kroshu_kuka_controllers&metric=alert_status)](https://sonarcloud.io/dashboard?id=kroshu_kuka_controllers)

The controllers in this repository can be divided into three categories.

## Traditional controllers

These controllers update the command interfaces of a hardware cyclically.

#### `joint_group_impedance_controller`
The joint impedance controller listens on the `~/command` topic and updates the `stiffness` and `damping` interfaces of the hardware accordingly.
The command must be of `std_msgs::Float64MultiArray` type and must contain the values for all configured joints. The order of the values should match the `stifness_1, damping_1, stiffness_2, ...` pattern.

Example cli command to set damping to 0.7 and stiffness to 100 for all joints of a 6 DOF robot:

```
ros2 topic pub /joint_group_impedance_controller/commands std_msgs/msg/Float64MultiArray "{data: [100, 0.7 ,100 ,0.7, 100, 0.7, 100, 0.7, 100, 0.7, 100, 0.7]}" --once
```

__Required parameters__:
- `joints` [string_array]: Names of joints used by the controller

#### `effort_controllers`
This package is available in ros2_controllers repo, however the `JointGroupPositionController` was not ported to humble, at the time of writing only a [draft PR](https://github.com/ros-controls/ros2_controllers/pull/198) exists for it. To add this functionality, contents of the PR were copied here.

The `effort_controllers/JointGroupEffortController` is a command forwarding controller for torque command interfaces. It accepts torque input on the `~/command` topic and sets the configured torque command interfaces without checking for limits. 

__Required parameters__:
- `joints` [string_array]: Names of joints used by the controller

The `effort_controllers/JointGroupPositionController` implements a PID controller: it accepts position input and calculates the needed torques according to the PID parameters for every controlled joint

__Required parameters__:
- `joints` [string_array]: Names of joints used by the controller
- `gains/joint_name/p` [double]: Proportianal term of the controller for *joint_name*
- `gains/joint_name/i` [double]: Integral term of the controller for *joint_name*
- `gains/joint_name/d` [double]: Derivative term of the controller for *joint_name*

## Broadcasters

Broadcasters receive the state interfaces of a hardware and publish it to a ROS2 topic.
#### `fri_state_broadcaster`

The `FRIStateBroadcaster` publishes the actual state of FRI to the `~/fri_state` topic, using the custom [FRIState](https://github.com/kroshu/kuka_drivers/blob/master/kuka_driver_interfaces/msg/FRIState.msg) message.

__Required parameters__: None

## Configuration controllers

Hardware interfaces do not support parameters that can be changed in runtime. To provide this behaviour, configuration controllers can be used, which update specific command interfaces of a hardware, that are exported as a workaround instead of parameters.

#### `control_mode_handler`

The `ControlModeHandler` can update the `control_mode` command interface of a hardware. It listens on the `~/control_mode` topic and makes control mode changes possible without having to reactivate the driver.
The control mode is [defined as an enum](https://github.com/kroshu/kuka_drivers/blob/master/kuka_drivers_core/include/kuka_drivers_core/control_mode.hpp) in the `kuka_drivers_core` package, the subscription therefore is of an unsigned integer type.

__Required parameters__: None

#### `fri_configuration_controller`

The `receive_multiplier` parameter of FRI defines the answer rate factor (ratio of receiving states and sending commands). This is a parameter of the hardware interface, which can be modified in connected state, when control is not active. To support changing this parameter after startup, the `FRIConfigurationController` implementes a service named `~/set_receive_multiplier`. Sending a request containing the desired integer value of the `receive_multiplier` updates the parameter of the hardware interface.

__Required parameters__: None