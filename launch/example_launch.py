#!/usr/bin/env python3
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).lower() in {"1", "true", "yes", "on"}


def _strip_xml_declaration(xml):
    stripped = xml.lstrip()
    if stripped.startswith("<?xml"):
        return stripped.split("?>", 1)[1].lstrip()
    return xml


def _load_robot_description(xacro_path):
    xml = subprocess.check_output(["xacro", str(xacro_path)], text=True)
    return _strip_xml_declaration(xml)


def _write_spawn_urdf(entity_name, robot_description):
    path = Path(tempfile.gettempdir()) / f"gazebo_continuous_track_ros2_gazebo11_{entity_name}.urdf"
    path.write_text(robot_description, encoding="utf-8")
    return path


def _robot_include_name(include_elem):
    uri_elem = include_elem.find("uri")
    if uri_elem is None or uri_elem.text is None:
        return None
    uri = uri_elem.text.strip()
    prefix = "model://"
    if not uri.startswith(prefix):
        return None
    name = uri[len(prefix):]
    return name if name.startswith("example_") else None


def _sanitize_world(world_path, robots):
    tree = ET.parse(world_path)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        return world_path

    robot_names = {robot["entity"] for robot in robots}
    for include in list(world.findall("include")):
        include_name = _robot_include_name(include)
        if include_name in robot_names:
            world.remove(include)

    for model in list(world.findall("model")):
        if model.attrib.get("name") == "anchor":
            world.remove(model)

    sanitized = Path(tempfile.gettempdir()) / f"gazebo_continuous_track_ros2_gazebo11_{world_path.stem}.world"
    tree.write(sanitized, encoding="unicode", xml_declaration=True)
    return sanitized


def _robot_actions(robot, example_share, use_sim_time):
    entity = robot["entity"]
    xacro_path = example_share / "urdf_xacro" / robot["xacro"]
    robot_description = _load_robot_description(xacro_path)
    spawn_urdf = _write_spawn_urdf(entity, robot_description)

    x, y, z, roll, pitch, yaw = robot.get("pose", (0.0, 0.0, 0.3, 0.0, 0.0, 0.0))
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=entity,
            name="state_publisher",
            output="log",
            parameters=[
                {
                    "robot_description": robot_description,
                    "use_sim_time": use_sim_time,
                    "frame_prefix": f"{entity}/",
                }
            ],
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name=f"{entity}_spawner",
            output="screen",
            arguments=[
                "-entity",
                entity,
                "-file",
                str(spawn_urdf),
                "-x",
                str(x),
                "-y",
                str(y),
                "-z",
                str(z),
                "-R",
                str(roll),
                "-P",
                str(pitch),
                "-Y",
                str(yaw),
                "-timeout",
                "120",
            ],
            additional_env={"PATH": f"/usr/bin:{os.environ.get('PATH', '')}"},
        ),
    ]


def generate_example_launch_description(world_name, robots, start_robot_steering_default="false"):
    def _launch_setup(context, *args, **kwargs):
        example_share = Path(get_package_share_directory("gazebo_continuous_track_example_ros2_gazebo11"))
        example_prefix = example_share.parent.parent
        track_prefix = Path(get_package_share_directory("gazebo_continuous_track_ros2_gazebo11")).parent.parent

        world = _sanitize_world(example_share / "world" / world_name, robots)
        gzserver_cmd = [
            "gzserver",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            "-s",
            "libgazebo_ros_state.so",
            str(world),
        ]

        paused = _as_bool(LaunchConfiguration("paused").perform(context))
        if paused:
            gzserver_cmd.append("--pause")

        actions = [
            SetEnvironmentVariable(
                "GAZEBO_PLUGIN_PATH",
                f"{example_prefix / 'lib'}:{track_prefix / 'lib'}:{os.environ.get('GAZEBO_PLUGIN_PATH', '')}",
            ),
            ExecuteProcess(cmd=gzserver_cmd, output="screen"),
        ]

        if _as_bool(LaunchConfiguration("gui").perform(context)):
            actions.append(ExecuteProcess(cmd=["gzclient"], output="screen"))

        if _as_bool(LaunchConfiguration("start_robot_steering").perform(context)):
            actions.append(
                Node(
                    package="rqt_robot_steering",
                    executable="rqt_robot_steering",
                    name="robot_steering",
                    output="screen",
                )
            )

        use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
        for robot in robots:
            actions.extend(_robot_actions(robot, example_share, use_sim_time))

        if paused:
            actions.append(
                TimerAction(
                    period=5.0,
                    actions=[ExecuteProcess(cmd=["gz", "world", "-m", "2"], output="screen")],
                )
            )

        return actions

    return LaunchDescription(
        [
            DeclareLaunchArgument("paused", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("start_robot_steering", default_value=start_robot_steering_default),
            OpaqueFunction(function=_launch_setup),
        ]
    )
