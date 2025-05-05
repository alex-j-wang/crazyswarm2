Outside container
```bash
ros2 launch motion_capture_tracking launch.py
ros2 topic list
```

Inside container
```bash
catkin build
source devel/setup.bash
```

Need `tf` outside container to communicate with `tf` inside container. Want `tf` to show up when we run `rostopic list`.

Ideas
- Use `ros2` motion capture tracking and try `ros1_bridge`.
- Clone https://github.com/IMRCLab/motion_capture_tracking/tree/main into container and run it. Set hostname to IP.

devel/share/motion_capture_tracking

https://docs.ros.org/en/humble/index.html
https://en.cppreference.com/w/

port IMU to ros2
- look at Crazyswarm2 Crazyflie.h
- look at crazyflie_ros - how does ros1 publish IMU code, turn into ros2
- CrazyflieROS class deals with node communication

use cpp API to get IMU data
translate data type to ros message
make or reuse publisher from crazyflie to server
display data on server

smart pointers?
pull request

make file src/imu_tester.cpp
imu_tester includes Crazyflie.h, runs a main function
include memory

inside main, make a unique ptr of type crazyflie my_cf
std::unique_ptr<Crazyflie> my_cf = 

("radio://")
std::cout << "Got imu reading: "%f" << my_cf->function

else of CMakeLists.txt
```cmake
add_executable(crazyflie_imu_tester, src/imu_tester.cpp)
target_link_libraries(crazyflie_imu_tester crazyflie_cpp)
```

mkdir build
cmake -B build -S .
cmake -B build -S .

crazyswarm2
- crazyflie.py:488
- teleop.cpp:164
- crazyflie_server.cpp:727

crazyflie_ros
- crazyflie_server.cpp:609
- crazyflie_server.cpp:504

# Questions

crazyswarm2
Purpose of `teleop.cpp`? Namely `publish` seems to get some position data. But `crazyflie_server.cpp` has an `on_logging_pose` function. Does `publish` somehow set the `data` input to `on_logging_pose`?

crazyflie_ros
Is `onPoseData` equivalent to crazyswarm2 `on_logging_pose`? And would my ultimate goal be to add a parallel to `onImuData`? Also, it seems like IMU data has a dedicated topic `m_pubImu`. Is that how the system should be set up?

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
ros2 launch '/home/anoop/Alex/crazyswarm2/crazyflie/launch/launch.py'
ros2 topic list
ros2 topic echo
ros2 bag record

ps axf | grep docker | grep -v grep | awk '{print "kill -9 " $1}' | sudo sh
systemctl --user start docker-desktop
make run
source /opt/ros/humble/setup.sh
rm -rf build install log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
source install/local_setup.bash
ros2 launch crazyflie_mpc launch.py
ros2 launch crazyflie launch.py

TODO:
- Check Docker build log, see if anything in `requirements.txt` is unnecessary
- Update crazyflie_mpc CMakeLists.txt and package.xml
- Update python scripts
- Fix NODE import
- Make sure crazyflie/joystick nodes work
- Check over README
- Transition to crazyswarm2 API
- Use common config file(s) / take config params from `crazyflies.yaml`
- Understand all magic numbers and add to config (including model filenames)
- Add server to launch file(s)

figure out Docker crashing when detaching on Linux
transition Makefile to bash scripts
GeometriControl
go through with a formatter

ros2 launch crazyflie_mpc trajectory_launch.py trajectory:=figure8
ros2 launch crazyflie_mpc trajectory_launch.py trajectory:=circle controller:=geometric

notes for how to launch
get rid of try / except blocks

don't hard code m_state

Goals:
- Make a template MPC Python program
- API for inputs and outputs
- Easy to hook up any controller and test on prescribed trajectory

zoxide
access data package more elegantly!!!

`export PYTHONPATH="/ros_ws/src/crazyflie_mpc/data:$PYTHONPATH"`
no point specfying steps for linear trajectory
waypoint_traj deal with repeat points
takeoff, press key to land (geometric)

ros launch service

- Make `trajectories.yaml` specify per-crazyflie trajectories. The `trajectory_launch.py` script should look at both `crazyflies.yaml` and `trajectories.yaml` and assert a sane state before launching all nodes. The new trajectory format should be
```
cf18:
  type: linear
  params:
    start: [x, y, z]
    end: [x, y, z]
```
- Start a bunch of crazyflies (loop in trajectory_launch?).
- Either automatically or through command line, broadcast on `state_pub`. This should change all `m_state`s to 2.
- Either automatically or through command line, broadcast on `automaticService`. This should change all `m_state`s to 1. Automation would require polling that all crazyflies are ready.
- Either automatically or through command line, broadcast on `landingService`. This should change all `m_state`s to 3. Automation would require polling that all crazyflies are ready.
- Once a crazyflie is done landing, it should switch to idle.

- Get rid of trajectories.yaml
- Specify all trajectories in mpc_config.yaml
