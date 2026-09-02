import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory("my_robot_canopen"), "config", "my_robot"
    )

    device_container = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_core"), "launch"),
                "/canopen.launch.py",
            ]
        ),
        launch_arguments={
            "master_config": os.path.join(config_dir, "master.dcf"),
            "master_bin": "",
            "bus_config": os.path.join(config_dir, "bus.yml"),
            "can_interface_name": "can0",
        }.items(),
    )

    return LaunchDescription([device_container])