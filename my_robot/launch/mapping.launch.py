import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('my_robot')
    slam_params = os.path.join(pkg, 'config', 'slam_params.yaml')

    # 하부 구동 계층 선택: 'stm32'(CANopen) 또는 'arduino'(Serial)
    base = LaunchConfiguration('base')
    use_stm32 = PythonExpression(["'", base, "' == 'stm32'"])
    use_arduino = PythonExpression(["'", base, "' == 'arduino'"])

    return LaunchDescription([
        DeclareLaunchArgument(
            'base', default_value='stm32',
            description="하부 구동 계층: 'stm32'(STM32+CANopen) 또는 'arduino'(Arduino Serial)"),

        # 하드웨어 + 융합 + 조이스틱 — 둘 중 하나만 실행됨
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'bringup_real_stm32.launch.py')),
            condition=IfCondition(use_stm32),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'bringup_real_arduino.launch.py')),
            condition=IfCondition(use_arduino),
        ),

        # SLAM
        Node(
            package='slam_toolbox', executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_params],
            output='screen',
        ),
    ])
