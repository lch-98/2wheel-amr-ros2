import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('my_robot_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'my_robot.urdf.xacro')
    robot_desc = xacro.process_file(urdf_file).toxml()

    tb3_models_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='',
        description='Gazebo world file path'
    )
    world = LaunchConfiguration('world')

    return LaunchDescription([
        world_arg,

        # GAZEBO_MODEL_PATH 설정
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=tb3_models_path
        ),

        # Gazebo 실행
        ExecuteProcess(
            cmd=['gazebo', '--verbose',
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so',
                 world],
            output='screen'
        ),

        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc,
                         'use_sim_time': True}]
        ),

        # Gazebo에 로봇 소환
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description',
                       '-entity', 'my_robot',
                       '-x', '-2.0', '-y', '-0.5', '-z', '0.05'],
            output='screen'
        ),
    ])