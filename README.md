# robomotionsim-client

Python client library for controlling a robot in the RoboMotionSim simulation via ZeroMQ.

## Installation

**Requirements:** Python 3.10+, `pyzmq`

```bash
pip install pyzmq
```

Copy the `robomotionsim_client/` folder into your project, or add this repo to your Python path.

## Quick Start

### Non-blocking loop pattern

```python
from robomotionsim_client import Robot

robot = Robot("tcp://localhost:5555", "tcp://localhost:5556")
robot.connect()

robot.move_by_distance_raw(distance_mm=500, x_speed=100, y_speed=0,
                           rotation_speed=0, front_rotation_speed=0)

while not robot.is_move_finished():
    robot.run()  # polls sockets, sends heartbeats, updates telemetry

    # read telemetry any time
    print(f"x={robot.get_distance_traveled_x():.1f} y={robot.get_distance_traveled_y():.1f}")

robot.disconnect()
```

### Blocking pattern

```python
from robomotionsim_client import Robot

robot = Robot("tcp://localhost:5555", "tcp://localhost:5556")
robot.connect()

robot.move_by_distance_raw(500, 100, 0, 0, 0)
robot.run_to_position()  # blocks until CMD_COMPLETE

robot.move_by_time_raw(2.0, 0, 50, 0, 0)
robot.run_to_position()

robot.disconnect()
```

## API Reference

### `Robot(command_address, telemetry_address)`

Create a robot client. Addresses are ZeroMQ endpoints, e.g. `"tcp://localhost:5555"`.

### Lifecycle

| Method | Description |
|---|---|
| `connect()` | Perform handshake with the sim. Raises `RobotConnectionError` on failure. |
| `disconnect()` | Send DISCONNECT and close sockets. |
| `run()` | Non-blocking update tick. Polls command/telemetry sockets, sends heartbeats. Must be called in a loop. |
| `run_to_position()` | Blocking loop — calls `run()` until the last sent command receives CMD_COMPLETE. |

### Status

| Method | Returns | Description |
|---|---|---|
| `is_move_finished()` | `bool` | `True` if the last sent command received CMD_COMPLETE. |

### Telemetry

All values are cached from the most recent telemetry frame.

| Method | Returns | Description |
|---|---|---|
| `get_distance_traveled_x()` | `float` | X distance traveled (mm) |
| `get_distance_traveled_y()` | `float` | Y distance traveled (mm) |
| `get_velocity_x()` | `float` | Local X velocity (mm/s) |
| `get_velocity_y()` | `float` | Local Y velocity (mm/s) |
| `get_front_angle()` | `float` | Front angle (rad) |
| `get_chassis_angle()` | `float` | Chassis angle (rad) |
| `get_chassis_angular_velocity()` | `float` | Chassis angular velocity (rad/s) |
| `get_wheel_speed(wheel_index)` | `float` | Speed of wheel at index |
| `get_wheel_distance(wheel_index)` | `float` | Distance traveled by wheel at index |

### Movement Commands

All commands send immediately and return. Use `run()` / `run_to_position()` to process responses.

Speeds are in mm/s, distances in mm, angles in radians, times in seconds.

| Method | Description |
|---|---|
| `move_by_distance_raw(distance_mm, x_speed, y_speed, rotation_speed, front_rotation_speed)` | Move a distance with raw speeds |
| `move_by_time_raw(time_s, x_speed, y_speed, rotation_speed, front_rotation_speed)` | Move for a time with raw speeds |
| `move_at_speed_raw(x_speed, y_speed, rotation_speed, front_rotation_speed)` | Set continuous raw speed (no auto-stop) |
| `stop()` | Immediately stop all motors |
| `move_at_speed_motors(speeds)` | Set per-motor speeds. `speeds` is a `list[float]`. |
| `run_motor_for_time(motor_id, speed, time_s)` | Run one motor for a duration |
| `run_motor_for_distance(motor_id, speed, distance_mm)` | Run one motor for a distance |
| `stop_motor(motor_id)` | Immediately stop one motor |
| `start_motor(motor_id, speed)` | Start one motor at a speed |
| `move_by_time(time_s, x_speed, y_speed, rotation_speed, center_x_mm, center_y_mm, rotate_chassis)` | Move for a time with pivot point |
| `move_by_distance(distance_mm, x_speed, y_speed, rotation_speed, center_x_mm, center_y_mm, rotate_chassis)` | Move a distance with pivot point |
| `move_at_speed(x_speed, y_speed, rotation_speed, center_x_mm, center_y_mm, rotate_chassis)` | Set continuous speed with pivot point |
| `move_by_angle(x_speed, y_speed, angle_rad, rotation_speed, center_x_mm, center_y_mm, rotate_chassis)` | Rotate by an angle with pivot point |
| `move_by_angle_raw(angle_rad, x_speed, y_speed, rotation_speed, front_rotation_speed)` | Rotate by an angle with raw speeds |
| `clear_command_queue()` | Clear all queued commands on the sim |

### Exceptions

| Exception | When |
|---|---|
| `RobotConnectionError` | Handshake fails or times out |
| `RobotCommandError` | Sim returns CMD_ERROR for a command |
