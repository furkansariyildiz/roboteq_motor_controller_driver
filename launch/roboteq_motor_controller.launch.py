import os
from launch import LaunchDescription
from launch_ros.actions import SetParameter
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roboteq_motor_controller',
            executable='roboteq_motor_controller_node',
            name='roboteq_motor_controller_node',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('roboteq_motor_controller'),
                'config', 'config.yaml')]
        )
    ])