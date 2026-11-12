"""Push-cube sandbox: spawn_robot brings up Gazebo and the UR5e, this package records the task.

Teleop needs a real terminal, so start it separately:

    ros2 run gz_pushcube_nav keyboard_teleop
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    spawn_robot = get_package_share_directory("spawn_robot")
    this_pkg = get_package_share_directory("gz_pushcube_nav")

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(spawn_robot, "launch", "spawn_pushcube.launch.py"),
        ),
        launch_arguments={
            "use_sim_time": "True",
        }.items(),
    )
    collect = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(this_pkg, "launch", "collect.launch.py"),
        ),
        launch_arguments={
            "output_dir": LaunchConfiguration("output_dir"),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "output_dir",
            default_value="~/episodes_stage1",
            description="Directory for episode_XXXXXX.h5 files",
        ),
        bringup,
        collect,
    ])
