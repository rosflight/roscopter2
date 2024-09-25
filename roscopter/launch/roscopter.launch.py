import os
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    roscopter_dir = get_package_share_directory('roscopter')

    param_file = os.path.join(roscopter_dir, 'params', 'multirotor.yaml')
    controller_type = 'default'

    for arg in sys.argv:
        if arg.startswith("controller:="):
            controller_type = arg.split(':=')[1]

    return LaunchDescription([
        Node(
            package='roscopter',
            executable='controller',
            name='autopilot',
            output='screen',
            arguments=[controller_type],
            parameters=[param_file],
            remappings=[('estimated_state', 'state')]
        ),
        Node(
            package='roscopter',
            executable='trajectory_follower',
            name='trajectory_follower',
            output='screen',
            parameters=[param_file],
            remappings=[('estimated_state', 'state')]
        ),
        # Node(
        #     package='roscopter',
        #     executable='ekf_node',
        #     name='estimator'
        # )
        # Node(
        #     package='roscopter',
        #     executable='path_manager',
        #     name='path_manager',
        #     output='screen',
        #     remappings=[('estimated_state', 'state')]
        # ),
    ])
