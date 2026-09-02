import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('my_robot')
    ekf_params = os.path.join(pkg, 'config', 'ekf.yaml')

    arduino_port = LaunchConfiguration('arduino_port')
    lidar_port = LaunchConfiguration('lidar_port')

    return LaunchDescription([
        DeclareLaunchArgument('arduino_port', default_value='/dev/arduino'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ydlidar'),

        # 1. URDF → TF (robot_state_publisher)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'robot_state.launch.py'))
        ),

        # 2. base_controller — TF 발행 끔 (EKF가 대신 발행)
        Node(
            package='my_robot', executable='base_controller.py',
            name='base_controller',
            parameters=[{'port': arduino_port, 'publish_tf': False}],
            remappings=[('cmd_vel', '/cmd_vel_out')],     # ← 추가
            output='screen',
        ),

        # 3. IMU
        Node(
            package='my_robot', executable='imu_node.py',
            name='imu_node',
            output='screen',
        ),

        # 4. EKF — odom → base_footprint TF 발행
        Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_params],
            output='screen',
        ),

        # 5. 라이다
        Node(
            package='my_robot', executable='ydlidar_node',
            name='ydlidar_node',
            parameters=[{'port': lidar_port}],
            output='screen',
        ),

        # 6. 조이스틱 teleop
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'joy_teleop.launch.py'))
        ),
    ])