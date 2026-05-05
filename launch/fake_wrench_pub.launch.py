import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory("force_controller"), "config", "fake_wrench_pub.yaml"
    )
      
    return LaunchDescription(
        [
            Node(
                package='force_controller',
                executable='fake_wrench_publisher',
                name='wrench_publisher',
                output='screen',
                parameters=[config]
            )
        ]
    )
