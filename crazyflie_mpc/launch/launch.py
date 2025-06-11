import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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

    with open(crazyflies_yaml_path, 'r') as f:
        crazyflies = yaml.safe_load(f)
    with open(mpc_yaml_path, 'r') as f:
        mpc = yaml.safe_load(f)
        
    world_frame = 'world_frame'
    mpc_demo_nodes = []
    static_tf_nodes = []

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
                    {'world_frame': world_frame},
                    {'frame': key},
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

    # Command node
    mpc_demo_nodes.append(Node(
        package='crazyflie_mpc',
        executable='command.py',
        name='command',
        namespace='command',
        parameters=[
            {'cfnames': [key for key in crazyflies['robots'].keys() if crazyflies['robots'][key]['enabled']]},
        ],
        output='screen'
    ))
    
    return LaunchDescription(mpc_demo_nodes + static_tf_nodes)