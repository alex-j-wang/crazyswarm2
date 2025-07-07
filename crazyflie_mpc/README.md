# Crazyflie MPC

ROS2 package for running model predictive control on Crazyflie quadrotors.

## Overview

This package provides MPC (Model Predictive Control) and other control algorithms for trajectory tracking using the Bitcraze Crazyflie quadrotor. The controllers include:

- Geometric Controller
- Gaussian Process (GP) Controller
- Model Predictive Controller (MPC)
- Knowledge-Based Neural Ordinary Differential Equation (KNODE) Controller

## Prerequisites

Docker must be installed on your system. For the container to access your USB bus, Linux is recommended. Requirements should be downloaded by the Dockerfile. This package interfaces with [crazyswarm2](https://github.com/IMRCLab/crazyswarm2) running on ROS2 Humble.

## Installation

Clone this repository into any folder:

```bash
cd ~/path-to-some-folder/
git clone https://github.com/alex-j-wang/crazyswarm2.git
```

Build the Docker container:

```bash
cd crazyswarm2
./build-container
```

This step might take a while to complete. Once finished, run the container:

```bash
./run-container
```

The container will detach. You can now connect to it using VSCode Dev Containers or through the command line:

```bash
./join-container
```

Navigate to the `src` directory and build the packages:

```bash
cd src
make
```

Installation should be complete.

## Usage

### Basic Demo

To run the MPC controller, use:

```bash
ros2 launch crazyflie launch.py gui:=False teleop:=False reboot:=True
ros2 launch crazyflie_mpc launch.py
```

These can be performed in any directory once the packages are built but must be run in separate command line instances. Closed-loop control requires a camera system publishing `tf` updates to the ROS system.

### Controller Parameters

Crazyflies must be enabled in `/config/crazyflies.yaml` under the `crazyflie` package. Ensure uri settings are correct. A blinking red LED indicates a Crazyflie is connected.

You can configure controller parameters in `/config/mpc.yaml`. Each active Crazyflie should have a trajectory specified here. Trajectory formats are demonstrated in `/config/example_trajectories.yaml`.

To test `crazyflie_mpc` without hardware, launch with `sim:=True`. This will silence warnings and suppress velocity commands for debugging purposes.

### Plotting

Plotting of actual and prescribed trajectories can be enabled through either `/config/mpc.yaml` or `plotting:=True` when launching. For the GUI to appear, you must run `xhost +local:root` outside the container. If problems arise, ensure `echo $DISPLAY` is consistent inside and outside.

### Online Learning

When using the KNODE controller, the system will launch nodes to perform online learning based on live flight data. Initial models for each Crazyflie must be placed in `crazyflie_mpc/data/knode_models/init`. The training nodes will periodically publish new models to `crazyflie_mpc/data/knode_models/online`. Each Crazyflie has its own models.

## Topics

### Subscribed Topics

- `imu` (sensor_msgs/Imu): IMU data from the Crazyflie

### Published Topics

- `est_vel` (geometry_msgs/TwistStamped): Estimated velocity
- `u_euler` (geometry_msgs/TwistStamped): ???
- `cmd_vel_stamped` (geometry_msgs/Twist): Time-stamped velocity command
- `cmd_vel_legacy` (geometry_msgs/Twist): Velocity command sent to the Crazyflie
- `goal` (geometry_msgs/TwistStamped): Current goal point along the trajectory
- `tf_pos` (geometry_msgs/PoseStamped): Position from TF

### Other Topics

The package uses two internal topics to keep behavior synchronized:
- `cmd_state` (std_msgs/Int32): Phase requests from command node
- `cf_ready` (std_msgs/String): Readiness broadcasts from Crazyflie nodes
- `model` (std_msgs/Int32): Model update notifications from online learning nodes