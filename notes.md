https://docs.ros.org/en/humble/index.html
https://en.cppreference.com/w/

TODO:
- Update README
- Multi-step trajectories
- Give each trajectory step an optional duration field; speed defaults to the maximum velocity parameter
- Collision detection and avoidance for trajectories?
- `waypoint_traj` deal with repeat points (may be fixed by adding durations)

FUTURE PLANS:
- Fix casadi warnings... downgrade package?
- Use common config file(s) / take config params from `crazyflies.yaml`
- Understand all magic numbers and add to config (including model filenames)
- Check Docker build log, see if anything in `requirements.txt` is unnecessary
- Set up zoxide
- Update crazyflie_mpc CMakeLists.txt and package.xml?
- Add server to launch file(s)?
- Access MPC data package more elegantly?
- Make a template MPC Python program, API for inputs and outputs
- ~~Transition to crazyswarm2 API~~ (using `cmd_vel_legacy` switches to low-level control)
- Very long term: ideally we get drones with cameras that can see each other, bypassing the need for external motion tracking

LAUNCH:
- `ros2 launch crazyflie launch.py`
- `ros2 launch crazyflie_mpc launch.py`

USEFUL COMMANDS:
- `ros2 topic list`
- `ros2 topic echo`
- `ros2 bag record`

CLEANUP:
- Remove unused variables
- Consistent spacing, remove unnecessary whitespace
- Get rid of try / except blocks
- Refactoring (remove unnecessary variables/functions)

README NOTES:
- Trajectory format should be
  ```
  cf18:
    type: linear
    start: [x, y, z]
    end: [x, y, z]
  ```
- `./build-container`
- `./run-container`
- `./join-container`
- `make` (while in `src` directory)

MISCELLANEOUS / UNTESTED:
- look through ROS documentation, docs files
- builtin_interfaces/Duration duration
- ros2 run crazyflie reboot --uri radio://0/80/2M/E7E7E7E706
- ros2 run crazyflie_examples nice_hover
- self.create_service(Takeoff, "all/takeoff", self._takeoff_callback)
- command node broadcasts phase requests on cmd_state
- crazyflie nodes broadcast readiness on cf_ready
