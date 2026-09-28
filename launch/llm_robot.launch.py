#!/usr/bin/env python3
"""Launch UR3 Gazebo simulation (table + red/yellow/blue cubes) + MoveIt 2 +
skill_server (home/pick/place services). task_runner.py is run separately,
once this is up, for each natural-language command -- see README.

Reuses ur_simulation_gz's ur_sim_control.launch.py and ur_moveit_config's
ur_moveit.launch.py unmodified, the same way letter_writer's launch file
does (see that package's launch file docstring for the macOS/RoboStack
xacro-concurrency workaround this also relies on).
"""

import glob
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _macos_python_preload_env():
    """RoboStack/macOS workaround -- see letter_writer's launch file for the
    full explanation. No-op on Linux."""
    if sys.platform != "darwin" or "CONDA_PREFIX" not in os.environ:
        return {}
    candidates = glob.glob(os.path.join(os.environ["CONDA_PREFIX"], "lib", "libpython3.*.dylib"))
    return {"DYLD_INSERT_LIBRARIES": candidates[0]} if candidates else {}


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    startup_delay = LaunchConfiguration("startup_delay")

    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type", default_value="ur3", description="Type of UR robot to simulate."
        ),
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="false",
            description="Start Gazebo with its own GUI window (Linux only).",
        ),
        DeclareLaunchArgument(
            "startup_delay",
            default_value="15.0",
            description="Seconds to wait for Gazebo/controllers/MoveIt before starting skill_server.",
        ),
    ]

    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "gazebo_gui": gazebo_gui,
            "launch_rviz": "false",
            "world_file": PathJoinSubstitution(
                [FindPackageShare("ur3_llm_control"), "worlds", "tabletop.sdf"]
            ),
            "controllers_file": PathJoinSubstitution(
                [FindPackageShare("ur3_llm_control"), "config", "ur_controllers_relaxed.yaml"]
            ),
        }.items(),
    )

    ur_moveit = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "launch",
                    "ur_moveit_config",
                    "ur_moveit.launch.py",
                    ["ur_type:=", ur_type],
                    "use_sim_time:=true",
                    "launch_rviz:=false",
                ],
                output="screen",
                name="ur_moveit_launch",
            )
        ],
    )

    # Our own RViz, with a custom config that adds a "Scene (objects +
    # zones)" Marker display subscribed to skill_server's /scene_markers --
    # Gazebo has no usable GUI on macOS, and without this RViz would show
    # only the robot and an empty table, not what it's picking/placing.
    rviz = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_ur3_llm_control",
                output="log",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [FindPackageShare("ur3_llm_control"), "config", "ur3_llm_control.rviz"]
                    ),
                ],
                parameters=[{"use_sim_time": True}],
            )
        ],
    )

    skill_server_node = Node(
        package="ur3_llm_control",
        executable="skill_server",
        name="skill_server",
        output="screen",
        parameters=[
            PathJoinSubstitution([FindPackageShare("ur3_llm_control"), "config", "scene.yaml"]),
            {"use_sim_time": True},
        ],
        additional_env=_macos_python_preload_env(),
    )

    delayed_skill_server = TimerAction(period=startup_delay, actions=[skill_server_node])

    return LaunchDescription(
        declared_arguments + [ur_control, ur_moveit, rviz, delayed_skill_server]
    )
