import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.logging import get_logger
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
    rviz_config_path = os.path.join(
        get_package_share_directory('crazyflie_mpc'),
        'config',
        'trajectory_plotter.rviz')

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
                    },
                    mpc['constants'],
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

    # Command node
    mpc_demo_nodes.append(Node(
        package='crazyflie_mpc',
        executable='command.py',
        name='mpc_command',
        namespace='command',
        parameters=[{ 'cfnames': cfnames }],
        output='screen'
    ))

    # Plotting node
    mpc_demo_nodes.append(Node(
        package='crazyflie_mpc',
        executable='trajectory_plotter.py',
        name='mpc_plotter',
        parameters=[
            {
                'world_frame': world_frame,
                'cfnames': cfnames,
            },
            mpc['plotting'],
        ],
        condition=IfCondition(mpc['plotting']['enabled']),
        output='screen'
    ))

    # RViz node
    mpc_demo_nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='mpc_rviz',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(mpc['plotting']['enabled'] and mpc['plotting']['use_rviz'])
    ))
    
    return LaunchDescription(mpc_demo_nodes + static_tf_nodes)