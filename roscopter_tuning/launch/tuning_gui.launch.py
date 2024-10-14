import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    roscopter_tuning_dir = get_package_share_directory('roscopter_tuning')

    tuning_config = os.path.join(
        roscopter_tuning_dir,
        'resources',
        'param_tuning_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='rosflight_rqt_plugins',
            executable='param_tuning',
            name='tuning_gui',
            output='screen',
            arguments=['--config-filepath', tuning_config]
        )
    ])

