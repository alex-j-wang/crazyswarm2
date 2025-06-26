# Crazyflie MPC

ROS2 package for running model predictive control on Crazyflie quadrotors.

## Overview

This package provides MPC (Model Predictive Control) and other control algorithms for trajectory tracking using the Bitcraze Crazyflie quadrotor. The controllers include:

- Geometric Controller
- Gaussian Process (GP) Controller
- Hybrid Controller
- Model Predictive Controller (MPC)

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

These can be performed in any directory once the packages are built.

### Controller Parameters

Crazyflies must be enabled in `/config/crazyflies.yaml` under the `crazyflie` package.

You can configure controller parameters in `/config/mpc.yaml`. Each active Crazyflie should have a trajectory specified here. Trajectory formats are demonstrated in `/config/example_trajectories.yaml`.

To test `crazyflie_mpc` without hardware, launch with `sim:=True`. 

### Plotting

Plotting of actual and prescribed trajectories can be enabled through either `/config/mpc.yaml` or `plotting:=True` when launching. For the GUI to appear, you must run `xhost +local:root` outside the container.

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
