cmake_minimum_required(VERSION 3.8)
project(crazyflie_mpc)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  crazyflie_mpc/follow_waypoint.py
  crazyflie_mpc/geometric_control.py
  crazyflie_mpc/gp_control.py
  crazyflie_mpc/hybrid_control.py
  crazyflie_mpc/mpc_control.py
  crazyflie_mpc/waypoint_traj.py
  crazyflie_mpc/models.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
