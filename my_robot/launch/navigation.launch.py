import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    pkg = get_package_share_directory('my_robot')
    nav2_params = os.path.join(pkg, 'config', 'my_nav2_params_real.yaml')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    map_yaml = LaunchConfiguration('map')

    # 하부 구동 계층 선택: 'stm32'(CANopen) 또는 'arduino'(Serial)
    base = LaunchConfiguration('base')
    use_stm32 = PythonExpression(["'", base, "' == 'stm32'"])
    use_arduino = PythonExpression(["'", base, "' == 'arduino'"])

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser('~/robot_ws/src/maps/real/my_real_map.yaml')),
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

        # Nav2 (기본대로 controller → /cmd_vel_nav → velocity_smoother → /cmd_vel)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'use_sim_time': 'false',
                'map': map_yaml,
                'params_file': nav2_params,
            }.items()
        ),
    ])
