_NB: this was AI generated, no guarantees regarding its accuracy!_

# Crazyflie MPC

ROS2 package for model predictive control of Crazyflie quadrotors.

## Overview

This package provides MPC (Model Predictive Control) and other control algorithms for trajectory tracking with the Crazyflie quadrotor platform. The controllers include:

- Geometric Controller
- Model Predictive Controller (MPC)
- Hybrid Controller
- Gaussian Process (GP) Controller

## Prerequisites

- ROS2 Humble or newer
- Crazyflie ROS2 packages (`crazyflie`, `crazyflie_interfaces`)
- Python libraries: numpy, scipy

## Installation

Clone this repository into your ROS2 workspace:

```bash
cd ~/ros_ws/src
git clone https://github.com/your-username/crazyflie_mpc.git
cd ..
colcon build --packages-select crazyflie_mpc
source install/setup.bash
```

## Usage

### Basic Demo

To run the MPC controller with a simple trajectory:

```bash
ros2 launch crazyflie_mpc trajectory_launch.py
```

### Select Trajectory Type

The package comes with several predefined trajectory types:
- Circle
- Square
- Linear

You can select a trajectory type when launching:

```bash
ros2 launch crazyflie_mpc trajectory_launch.py trajectory:=circle
```

### Full Integration with Crazyflie

To use with a real Crazyflie and motion capture system:

```bash
ros2 launch crazyflie_mpc launch.py uri:=radio://0/80/2M/E7E7E7E7E7 frame:=cf1 use_mocap:=true
```

### Controller Parameters

You can configure controller parameters in:
- `/config/mpc_config.yaml`

Different trajectory parameters can be configured in:
- `/config/trajectories.yaml`

## Services

- Takeoff: Triggers the Crazyflie to take off
- Land: Triggers the Crazyflie to land

## Topics

### Subscribed Topics

- `/crazyflie/imu` (sensor_msgs/Imu): IMU data from the Crazyflie
- `/vicon/[frame_name]/pose` (geometry_msgs/PoseStamped): Pose data from motion capture

### Published Topics

- `cmd_vel` (geometry_msgs/Twist): Velocity commands for the Crazyflie
- `est_vel` (geometry_msgs/TwistStamped): Estimated velocity
- `goal` (geometry_msgs/TwistStamped): Current goal point along the trajectory
- `tf_pos` (geometry_msgs/PoseStamped): Position from TF

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Original ROS1 implementation by [original author]
- Adapted to ROS2 by [your name]