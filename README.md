# gazebo_continuous_track_example_ros2_gazebo11

ROS 2 examples for `gazebo_continuous_track_ros2_gazebo11`.

This is an unofficial ROS 2 / Gazebo 11 port of the ROS 1 example package
originally created by Okada et al. The original ROS 1 repositories are:

- https://github.com/yoshito-n-students/gazebo_continuous_track
- https://github.com/yoshito-n-students/gazebo_continuous_track_example

This package provides Gazebo Classic launch files and xacro models for comparing
continuous tracks, simple track approximations, simple wheel approximations, and
lugged wheels. The ROS 2 example track accepts `geometry_msgs/msg/Twist` on
`/cmd_vel` and animates the belt, pulley, and grouser geometry in Gazebo.

## Environment

Tested with:

- ROS 2 Humble
- Gazebo Classic 11
- Ubuntu 22.04

## Build

From the ROS 2 workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select gazebo_continuous_track_ros2_gazebo11 gazebo_continuous_track_example_ros2_gazebo11 --symlink-install
source install/setup.bash
```

## Launch

Start the main tracked vehicle example:

```bash
ros2 launch gazebo_continuous_track_example_ros2_gazebo11 example_track_world.launch.py
```

Start all comparison models:

```bash
ros2 launch gazebo_continuous_track_example_ros2_gazebo11 example_track_all_world.launch.py
```

Other available ROS 2 launch files:

- `example_track_simple_world.launch.py`
- `example_track_simple_wheels_world.launch.py`
- `example_lugged_wheel_world.launch.py`

Useful launch arguments:

- `paused:=true|false`
- `gui:=true|false`
- `use_sim_time:=true|false`
- `start_robot_steering:=true|false`

## Command Velocity

The main continuous-track model subscribes to `/cmd_vel`:

```bash
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.25}, angular: {z: 0.0}}"
```

`rqt_robot_steering` can also be started with:

```bash
ros2 launch gazebo_continuous_track_example_ros2_gazebo11 example_track_world.launch.py start_robot_steering:=true
```

## License

This ROS 2 / Gazebo 11 port keeps the original MIT License and copyright notice.
See `LICENSE` for the full license text. Keep that notice with redistributed
source or binary copies.

## Citation

Y. Okada, S. Kojima, K. Ohno and S. Tadokoro,
"Real-time Simulation of Non-Deformable Continuous Tracks with Explicit
Consideration of Friction and Grouser Geometry,"
2020 IEEE International Conference on Robotics and Automation (ICRA), 2020.
