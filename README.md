# offboard_control_cpp

ROS 2 C++ package for PX4 Offboard position control through `px4_msgs`.

This package provides two example nodes:

- `offboard_node`: flies through three fixed local-position waypoints.
- `simple_takeoff_node`: performs a simple takeoff, hover, and lateral move sequence.

The coordinate convention follows PX4 local NED:

- `+x`: forward
- `+y`: right
- `-z`: up

## Requirements

- ROS 2 Jazzy
- PX4 message package: `px4_msgs`
- PX4 running with ROS 2 bridge / Micro XRCE-DDS agent
- A configured ROS 2 workspace containing this package

## Package Layout

```text
offboard_control_cpp/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── offboard_control.launch.py
│   └── simple_takeoff.launch.py
└── src/
    ├── offboard_node.cpp
    └── simple_takeoff_node.cpp
```

## Build

From the workspace root:

```bash
cd ~/Documents/code/ros2_kc_ws
colcon build --symlink-install --packages-select offboard_control_cpp
source install/setup.bash
```

## Nodes

### `offboard_node`

Runs a three-waypoint state machine:

```text
Warmup
  -> FlyToFirstPoint:  (0.0, 0.0, -1.0)
  -> FlyToSecondPoint: (0.0, 1.5, -1.0)
  -> FlyToThirdPoint:  (1.5, 1.5, -1.0)
  -> Done
```

The node publishes:

- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/fmu/in/vehicle_command`

The node subscribes to:

- `/fmu/out/vehicle_local_position_v1` by default

Launch:

```bash
ros2 launch offboard_control_cpp offboard_control.launch.py
```

Launch with automatic Offboard mode and arming:

```bash
ros2 launch offboard_control_cpp offboard_control.launch.py auto_start:=true auto_arm:=true
```

Available launch arguments:

| Argument | Default | Description |
| --- | --- | --- |
| `auto_start` | `false` | Send PX4 command to enter Offboard mode after warmup. |
| `auto_arm` | `false` | Send PX4 arm command after warmup. |
| `warmup_setpoints` | `10` | Number of initial setpoints published before state transition. |
| `position_tolerance` | `0.1` | Position tolerance in meters for waypoint arrival. |
| `local_position_topic` | `/fmu/out/vehicle_local_position_v1` | Vehicle local position topic. |

### `simple_takeoff_node`

Runs a simple takeoff sequence:

```text
Warmup
  -> Takeoff
  -> Hover
  -> FlyRight
  -> Done
```

Launch:

```bash
ros2 launch offboard_control_cpp simple_takeoff.launch.py
```

Launch with automatic Offboard mode and arming:

```bash
ros2 launch offboard_control_cpp simple_takeoff.launch.py auto_start:=true auto_arm:=true
```

Available launch arguments:

| Argument | Default | Description |
| --- | --- | --- |
| `auto_start` | `false` | Send PX4 command to enter Offboard mode after warmup. |
| `auto_arm` | `false` | Send PX4 arm command after warmup. |
| `target_z` | `-1.0` | Takeoff height in local NED coordinates. |
| `next_target_x` | `0.0` | X coordinate of the next target. |
| `next_target_y` | `1.5` | Y coordinate of the next target. |
| `next_target_z` | `-1.0` | Z coordinate of the next target. |
| `hover_duration` | `5.0` | Hover duration in seconds before flying to the next target. |

## Manual PX4 Commands

If `auto_start` and `auto_arm` are left as `false`, the nodes still publish Offboard setpoints, but PX4 mode switching and arming must be done manually.

Example with PX4 shell:

```bash
commander mode offboard
commander arm
```

## Safety Notes

- Test in simulation before flying on real hardware.
- Make sure PX4 is receiving Offboard setpoints before switching to Offboard mode.
- Check that the local position topic matches your PX4/bridge setup.
- Keep `auto_arm:=false` until the full flow is verified.

## Git Remote

The package repository is intended to be pushed to:

```text
https://github.com/DDerer/offfboard_control_kc.git
```
