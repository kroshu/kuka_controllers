# kuka_controllers
ROS2 controllers necessary for KUKA robots

Github CI | SonarCloud
------------| ---------------
[![Build Status](https://github.com/kroshu/kuka_controllers/workflows/CI/badge.svg?branch=main)](https://github.com/kroshu/kuka_controllers/actions) | [![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=kroshu_kuka_controllers&metric=alert_status)](https://sonarcloud.io/dashboard?id=kroshu_kuka_controllers)

The controllers in this repository can be divided into three categories.

## Traditional controllers

These controllers update the command interfaces of a hardware cyclically.

#### `joint_impedance_controller`
The joint impedance controller listens on the `~/joint_impedance` topic and updates the `stiffness` and `damping` interfaces of the hardware accordingly.

#### `effort_controllers`


## Broadcasters

Broadcasters receive the state interfaces of a hardware and publish it to a ROS2 topic.
#### `fri_state_broadcaster`

## Configuration controllers

Hardware interfaces do not support parameters that can be changed in runtime. To provide this behaviour, configuration controllers can be used, which update specific command interfaces of a hardware, that are exported as a workaround instead of parameters.

#### `control_mode_handler`

####  `fri_configuration_controller`