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

    lidar_port = LaunchConfiguration('lidar_port')

    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value='/dev/ydlidar'),

        # 1. ros2_control(robot_state_publisher) + diff_drive_controller (CAN/CANopen) > 기존 base_controller를 대신해주는 역할
        #    TF 발행 끔 (EKF가 대신 발행), cmd_vel/odom 은 launch 내부에서 리매핑
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'robot_control.launch.py')),
        ),

        # 2. IMU
        Node(
            package='my_robot', executable='imu_node.py',
            name='imu_node',
            output='screen',
        ),

        # 3. EKF — odom → base_footprint TF 발행
        Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_params],
            output='screen',
        ),

        # 4. 라이다
        Node(
            package='my_robot', executable='ydlidar_node',
            name='ydlidar_node',
            parameters=[{'port': lidar_port}],
            output='screen',
        ),

        # 5. 조이스틱 teleop
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'joy_teleop.launch.py'))
        ),
    ])