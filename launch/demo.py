from launch import LaunchDescription , actions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os.path


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='smart_controller_gateway',
            executable='smart_controller_gateway_node',
            parameters=[{'is_pubulish_twist' : True}],
        ),
    ])
