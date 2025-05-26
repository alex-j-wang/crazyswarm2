https://docs.ros.org/en/humble/index.html
https://en.cppreference.com/w/

ros2 launch crazyflie launch.py
ros2 launch crazyflie_mpc launch.py

ros2 topic list
ros2 topic echo
ros2 bag record

make run
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
make build

TODO:
- Check Docker build log, see if anything in `requirements.txt` is unnecessary
- Update crazyflie_mpc CMakeLists.txt and package.xml
- Check over README
- Transition to crazyswarm2 API?
- Use common config file(s) / take config params from `crazyflies.yaml`
- Understand all magic numbers and add to config (including model filenames)
- Add server to launch file(s)?

transition Makefile to bash scripts
GeometriControl
go through with a formatter

notes for how to launch
get rid of try / except blocks

don't hard code m_state

Goals:
- Make a template MPC Python program
- API for inputs and outputs
- Easy to hook up any controller and test on prescribed trajectory

zoxide
access data package more elegantly!!!

no point specfying steps for linear trajectory
waypoint_traj deal with repeat points
takeoff, press key to land (geometric)

ros launch service

- Trajectory format should be
  ```
  cf18:
    type: linear
    start: [x, y, z]
    end: [x, y, z]
  ```
- Start a bunch of crazyflies (loop in trajectory_launch?).
- Either automatically or through command line, broadcast on `state_pub`. This should change all `m_state`s to 2.
- Either automatically or through command line, broadcast on `automaticService`. This should change all `m_state`s to 1. Automation would require polling that all crazyflies are ready.
- Either automatically or through command line, broadcast on `landingService`. This should change all `m_state`s to 3. Automation would require polling that all crazyflies are ready.
- Once a crazyflie is done landing, it should switch to idle.

how often do files really need to be rebuilt?

./build-container
./run-container
./join-container
