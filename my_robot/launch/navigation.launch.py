import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg = get_package_share_directory('my_robot')
    nav2_params = os.path.join(pkg, 'config', 'my_nav2_params_real.yaml')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    map_yaml = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser('~/robot_ws/src/maps/real/my_real_map.yaml')),

        # 하드웨어 + 융합 + 조이스틱
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'bringup_real.launch.py'))
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