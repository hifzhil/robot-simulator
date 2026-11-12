"""Gazebo Harmonic world plus UR5e-with-pusher bringup.

Uses the same split as spawn_robot.launch.py: world.launch.py starts Gazebo,
robot_state_publisher expands the xacro, ros_gz_sim create inserts the model,
and ros_gz_bridge exposes clock, the overhead camera, and dynamic poses.

Task nodes (teleop, success, recording) live in gz_pushcube_nav.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_spawn_robot = get_package_share_directory('spawn_robot')

    world_arg = DeclareLaunchArgument(
        'world', default_value='pushcube.sdf',
        description='Gazebo world file in the spawn_robot worlds directory'
    )
    model_arg = DeclareLaunchArgument(
        'model', default_value='ur_pusher.urdf.xacro',
        description='URDF/xacro file name under model/ur_pusher/urdf'
    )
    x_arg = DeclareLaunchArgument(
        'x', default_value='0.0',
        description='x coordinate of the spawned arm'
    )
    y_arg = DeclareLaunchArgument(
        'y', default_value='0.0',
        description='y coordinate of the spawned arm'
    )
    yaw_arg = DeclareLaunchArgument(
        'yaw', default_value='0.0',
        description='yaw of the spawned arm'
    )
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    urdf_file_path = PathJoinSubstitution([
        pkg_spawn_robot,
        'model',
        'ur_pusher',
        'urdf',
        LaunchConfiguration('model'),
    ])
    controllers_file = os.path.join(pkg_spawn_robot, 'config', 'ur_pusher_controllers.yaml')
    bridge_params = os.path.join(pkg_spawn_robot, 'config', 'pushcube_bridge.yaml')

    robot_description = Command([
        'xacro ', urdf_file_path,
        ' name:=ur',
        ' ur_type:=ur5e',
        ' safety_limits:=true',
        ' simulation_controllers:=', controllers_file,
    ])

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_spawn_robot, 'launch', 'world.launch.py'),
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
        }.items(),
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    spawn_urdf_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'ur',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', '0.0',
            '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )
    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_velocity_controller', '--controller-manager', '/controller_manager', '--param-file', controllers_file],
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # Gazebo has to be up before create, and the controller manager appears with the model.
    delayed_spawn = TimerAction(period=3.0, actions=[spawn_urdf_node])
    delay_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_urdf_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    delay_velocity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[velocity_controller_spawner],
        )
    )

    launch_description = LaunchDescription()
    launch_description.add_action(world_arg)
    launch_description.add_action(model_arg)
    launch_description.add_action(x_arg)
    launch_description.add_action(y_arg)
    launch_description.add_action(yaw_arg)
    launch_description.add_action(sim_time_arg)
    launch_description.add_action(world_launch)
    launch_description.add_action(robot_state_publisher_node)
    launch_description.add_action(gz_bridge_node)
    launch_description.add_action(delayed_spawn)
    launch_description.add_action(delay_controllers)
    launch_description.add_action(delay_velocity)
    return launch_description
