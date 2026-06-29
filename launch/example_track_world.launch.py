#!/usr/bin/env python3
import importlib.util
from pathlib import Path


def _load_common():
    path = Path(__file__).with_name("example_launch.py")
    spec = importlib.util.spec_from_file_location("gazebo_continuous_track_example_ros2_gazebo11_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def generate_launch_description():
    common = _load_common()
    return common.generate_example_launch_description(
        "example_track.world",
        [
            {
                "entity": "example_track",
                "xacro": "example_track_gazebo.urdf.xacro",
                "pose": (0.0, 0.0, 0.3, 0.0, 0.0, 0.0),
            }
        ],
        start_robot_steering_default="true",
    )
