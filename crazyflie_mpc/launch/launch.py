import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.conditions import LaunchConfigurationEquals
from launch.event_handlers import OnProcessExit
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

def generate_launch_description():
    logger = get_logger('launch')

    # Launch Arguments
    crazyflies_yaml_path = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'crazyflies.yaml')
    mpc_yaml_path = os.path.join(
        get_package_share_directory('crazyflie_mpc'),
        'config',
        'mpc.yaml')
    
    with open(crazyflies_yaml_path, 'r') as f:
        crazyflies = yaml.safe_load(f)
    with open(mpc_yaml_path, 'r') as f:
        mpc = yaml.safe_load(f)

    world_frame = 'world'
    mpc_demo_nodes = []
    static_tf_nodes = []
    cfnames = []

    for key, value in crazyflies['robots'].items():
        if value['enabled']:
            if key not in mpc['trajectories']:
                logger.error(f"Trajectory not specified for robot {key}")
                raise RuntimeError(f"no trajectory for robot {key}")

            # MPC demo node
            mpc_demo_nodes.append(Node(
                package='crazyflie_mpc',
                executable='follow_waypoint.py',
                name='mpc_demo',
                namespace=key,
                parameters=[
                    {
                        'world_frame': world_frame,
                        'frame': key,
                        'x_final': value['initial_position'][0],
                        'y_final': value['initial_position'][1],
                        'sim': LaunchConfiguration('sim'),
                    },
                    mpc['constants'],
                    mpc['variables'][key],
                    mpc['trajectories'][key]
                ],
                output='screen'
            ))

            # Static transform publisher
            static_tf_nodes.append(Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='world_broadcaster_' + key,  # Make the node name unique
                arguments=[
                    '--x', '0', '--y', '0', '--z', '0',
                    '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                    '--frame-id', world_frame,
                    '--child-frame-id', key
                ]
            ))

            cfnames.append(key)

    command_node = Node(
        package='crazyflie_mpc',
        executable='command.py',
        name='mpc_command',
        namespace='command',
        parameters=[{ 'cfnames': cfnames }],
        output='screen',
    )

    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=command_node,
            on_exit=[Shutdown(reason='trajectories complete')]
        )
    )

    plotting_node = Node(
        package='crazyflie_mpc',
        executable='trajectory_plotter.py',
        name='mpc_plotter',
        parameters=[
            {
                'world_frame': world_frame,
                'cfnames': cfnames,
            },
        ],
        condition=LaunchConfigurationEquals('plotting', 'True'),
        output='screen'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('sim', default_value='False'),
            DeclareLaunchArgument('plotting', default_value=str(mpc['constants']['plotting'])),
            command_node,
            shutdown_handler,
            plotting_node,
            *mpc_demo_nodes,
            *static_tf_nodes
        ]
    )