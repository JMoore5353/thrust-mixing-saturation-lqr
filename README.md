# Aggressive Quadrotor Flight
This repository is a ROS2 implementation of the 2017 RAL paper titled "Thrust Mixing, Saturation, and Body-Rate Control for Accurate Aggressive Quadrotor Flight" by Faessler, et. al.

It is implemented to work with the [ROScopter](https://github.com/rosflight/roscopter.git) controller.

## Building and running
Build and source this repository in a terminal (that has sourced ROS2) with
```
colcon build
source install/setup.zsh
```

Run with:
```
ros2 run thrust_mixing_lqr lqr_rate_controller
```
