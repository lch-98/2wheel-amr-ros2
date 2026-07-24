import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('my_robot')
    joy_params = os.path.join(pkg, 'config', 'joy_teleop.yaml')
    mux_params = os.path.join(pkg, 'config', 'twist_mux.yaml')

    return LaunchDescription([
        Node(package='joy', executable='joy_node',
             name='joy_node',
             parameters=[joy_params]),

        Node(package='teleop_twist_joy', executable='teleop_node',
             name='teleop_twist_joy_node',
             parameters=[joy_params],
             remappings=[('/cmd_vel', '/cmd_vel_joy')]),

        Node(package='twist_mux', executable='twist_mux',
             name='twist_mux',
             parameters=[mux_params],
             remappings=[('/cmd_vel_out', '/cmd_vel')]),
    ])