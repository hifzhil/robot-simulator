"""Success checker and episode recorder. Run keyboard_teleop in its own terminal."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    output_dir = DeclareLaunchArgument(
        "output_dir",
        default_value="~/episodes_stage1",
        description="Directory for episode_XXXXXX.h5 files",
    )
    sim = {"use_sim_time": True}
    return LaunchDescription([
        output_dir,
        Node(
            package="gz_pushcube_nav",
            executable="success_checker",
            output="screen",
            parameters=[sim],
        ),
        Node(
            package="gz_pushcube_nav",
            executable="episode_recorder",
            output="screen",
            parameters=[sim, {"output_dir": LaunchConfiguration("output_dir")}],
        ),
    ])
