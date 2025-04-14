import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('crazyflie_mpc')
    
    # Launch Arguments
    frame_arg = DeclareLaunchArgument('frame', default_value='crazyflie', description='Frame name')
    world_frame_arg = DeclareLaunchArgument('world_frame', default_value='world', description='World frame name')
    trajectory_arg = DeclareLaunchArgument('trajectory', default_value='circle', description='Trajectory type (circle, square, linear, figure8, spiral, hover)')
    controller_arg = DeclareLaunchArgument('controller', default_value='geometric', description='Controller type (geometric, mpc, hybrid, gp)')
    use_sim_arg = DeclareLaunchArgument('use_sim', default_value='true', description='Use simulation instead of real hardware')
    
    # Get launch configurations
    frame = LaunchConfiguration('frame')
    world_frame = LaunchConfiguration('world_frame')
    trajectory = LaunchConfiguration('trajectory')
    controller = LaunchConfiguration('controller')
    use_sim = LaunchConfiguration('use_sim')
    
    # Define config files
    controller_config = os.path.join(pkg_share, 'config', 'mpc_config.yaml')
    trajectory_config = os.path.join(pkg_share, 'config', 'trajectories.yaml')

    # MPC demo node with trajectory following
    mpc_demo_node = Node(
        package='crazyflie_mpc',
        executable='follow_waypoint.py',
        name='mpc_demo',
        namespace='crazyflie',
        parameters=[
            {'frame': frame},
            {'world_frame': world_frame},
            {'trajectory_type': trajectory},
            {'controller_type': controller},
            {'control_frequency': 100.0},
            controller_config,
            trajectory_config
        ],
        output='screen'
    )

    # Static transform publisher for world frame
    static_tf_world_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', '1',
                  'world', 'map']
    )

    # If using simulation, add a node to publish initial pose
    sim_pose_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sim_pose_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1',
                  world_frame, frame],
        condition=IfCondition(use_sim)
    )
    
    return LaunchDescription([
        # Launch arguments
        frame_arg,
        world_frame_arg,
        trajectory_arg,
        controller_arg,
        use_sim_arg,
        
        # Nodes
        mpc_demo_node,
        static_tf_world_node,
        sim_pose_publisher,
    ])