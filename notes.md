https://docs.ros.org/en/humble/index.html
https://en.cppreference.com/w/

TODO:
- Look at crazyflie_online (crazyflie_ros package)
- Figure out `rclpy.shutdown()` and reincorporate it to `command` and `follow_waypoint` nodes (try / except / finally)
    ```
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    ```
- Fix output when node killed while plotting

FUTURE PLANS:
- Refactor reboot command into a server callback
- Multi-step trajectories
- Give each trajectory step an optional duration field; speed defaults to the maximum velocity parameter
- Collision detection and avoidance for trajectories?
- `waypoint_traj` deal with repeat points (may be fixed by adding durations)
- Use common config file(s) / take config params from `crazyflies.yaml`
- Understand all magic numbers and add to config (including model filenames)
- Check Docker build log, see if anything in `requirements.txt` is unnecessary
- Set up zoxide
- Update crazyflie_mpc CMakeLists.txt and package.xml?
- Add server to launch file(s)?
- Access MPC data package more elegantly?
- Make a template MPC Python program, API for inputs and outputs
- Set up SIM backend (`cmd_vel_legacy` not yet implemented)
- ~~Transition to crazyswarm2 API~~ (using `cmd_vel_legacy` switches to low-level control)
- Very long term: ideally we get drones with cameras that can see each other, bypassing the need for external motion tracking

USEFUL COMMANDS:
- `ros2 topic list`
- `ros2 topic echo`
- `ros2 bag record`
- `ros2 run crazyflie_examples nice_hover`

CLEANUP:
- Consistency with declaring parameters, get_parameter_value()
- Remove unused variables
- Consistent spacing, remove unnecessary whitespace
- Get rid of try / except blocks
- Refactoring (remove unnecessary variables/functions)
- Remove unnecessary things like `self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value` (only really important if used more than once)
- Purpose of `cmd_vel_stamped`? Is this the same as `cmd_vel_legacy`?
- Code formatter

MISCELLANEOUS / UNTESTED:
look through ROS documentation, docs files
builtin_interfaces/Duration duration
ros2 run tf2_ros tf2_echo world_frame cf2

VICON SETUP
- Object name
- CTRL + ALT to select points
- Create
