import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('crazyflie_mpc')
    crazyflie_share = get_package_share_directory('crazyflie')
    
    # Launch Arguments
    uri_arg = DeclareLaunchArgument('uri', default_value='radio://0/80/2M', description='Crazyflie URI')
    frame_arg = DeclareLaunchArgument('frame', default_value='crazyflie', description='Frame name')
    world_frame_arg = DeclareLaunchArgument('world_frame', default_value='world', description='World frame name')
    joy_dev_arg = DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0', description='Joystick device')
    use_mocap_arg = DeclareLaunchArgument('use_mocap', default_value='True', description='Use motion capture')
    
    # Get launch configurations
    uri = LaunchConfiguration('uri')
    frame = LaunchConfiguration('frame')
    world_frame = LaunchConfiguration('world_frame')
    joy_dev = LaunchConfiguration('joy_dev')
    use_mocap = LaunchConfiguration('use_mocap')
    
    # Define config file
    config_path = os.path.join(pkg_share, 'config', 'mpc_config.yaml')

    # Include main crazyflie launch file
    crazyflie_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(crazyflie_share, 'launch', 'launch.py')
        ]),
        launch_arguments={
            'backend': 'cflib',  # Use cflib backend
            'teleop': 'false',   # Disable teleop
            'gui': 'false',      # Disable GUI
            'mocap': use_mocap
        }.items()
    )

    # MPC demo node
    mpc_demo_node = Node(
        package='crazyflie_mpc',
        executable='follow_waypoint.py',
        name='mpc_demo',
        namespace='crazyflie',
        parameters=[
            {'frame': frame},
            {'world_frame': world_frame},
            config_path
        ],
        output='screen'
    )

    # Joystick controller node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'device': joy_dev}],
        output='screen'
    )

    # Static transform publisher
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='baselink_broadcaster',
        namespace='crazyflie',
        arguments=['0', '0', '0', '0', '0', '0', '1',
                  frame, '/crazyflie/base_link']
    )

    # Motion capture launch (if enabled)
    mocap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('motion_capture_tracking'), 
                        'launch', 'launch.py')
        ]),
        condition=IfCondition(use_mocap)
    )

    return LaunchDescription([
        # Launch arguments
        uri_arg,
        frame_arg,
        world_frame_arg,
        joy_dev_arg,
        use_mocap_arg,
        
        # Launch files and nodes
        mocap_launch,
        crazyflie_launch,
        joy_node,
        mpc_demo_node,
        static_tf_node
    ])