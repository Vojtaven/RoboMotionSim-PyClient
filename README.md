# Robomotionsim-PyClient

Python client library for controlling a robot in the [RoboMotionSim](https://github.com/Vojtaven/RoboMotionSim) simulation over ZeroMQ.

- Simple, dependency-light: only requires `pyzmq`
- Non-blocking by design — you control the update loop
- Full telemetry access: position, velocity, wheel speeds, angles
- Per-motor and high-level movement commands

---

## Requirements

- Python 3.10+
- [`pyzmq`](https://pyzmq.readthedocs.io/)

---

## Installation

```bash
pip install pyzmq
```

Clone or copy the `robomotionsim_client/` folder into your project:

```
your_project/
├── robomotionsim_client/   ← copy here
│   ├── __init__.py
│   └── robot.py
└── your_script.py
```

Or add this repo to your Python path:

```bash
git clone https://github.com/Vojtaven/RoboMotionSim-PyClient
cd RoboMotionSim-PyClient
```

---

## Quick Start

### Connect to the simulation

```python
from robomotionsim_client import Robot

robot = Robot(
    command_address="tcp://localhost:5555",
    telemetry_address="tcp://localhost:5556",
)
robot.connect()   # raises RobotConnectionError if sim is not running
print("Connected!")
```

### Move and wait (blocking)

The simplest pattern — send a command and block until it completes:

```python
# Move forward 500 mm at speed 100 mm/s
robot.move_by_distance_raw(distance_mm=500, x_speed=100, y_speed=0,
                           rotation_speed=0, front_rotation_speed=0)
robot.run_to_position()  # blocks until the move is done

print(f"Arrived at x={robot.get_distance_traveled_x():.1f} mm")

# Turn 90 degrees
robot.move_by_angle_raw(angle_deg=90, x_speed=0, y_speed=0,
                        rotation_speed=50, front_rotation_speed=0)
robot.run_to_position()

robot.disconnect()
```

### Non-blocking loop (live telemetry)

Use this pattern to read telemetry while the robot moves:

```python
import time

robot.move_at_speed_raw(x_speed=80, y_speed=0, rotation_speed=0, front_rotation_speed=0)

start = time.monotonic()
while time.monotonic() - start < 5.0:
    robot.run()  # polls sockets, updates telemetry, sends heartbeats

    print(
        f"pos=({robot.get_distance_traveled_x():7.1f}, {robot.get_distance_traveled_y():7.1f})  "
        f"vel=({robot.get_velocity_x():6.1f}, {robot.get_velocity_y():6.1f})  "
        f"angle={robot.get_chassis_angle():6.1f} deg",
        end="\r",
    )
    time.sleep(0.05)

robot.stop()
robot.run_to_position()
robot.disconnect()
```

> **Important:** You must call `robot.run()` regularly in your loop. It drives all socket I/O, telemetry updates, and heartbeats. Without it, the simulation will eventually disconnect you.

---

## Examples

| File | Description |
|---|---|
| [`examples/basic_movement.py`](examples/basic_movement.py) | Move forward, turn, print position |
| [`examples/motor_control.py`](examples/motor_control.py) | Per-motor speed and distance control |
| [`examples/telemetry_loop.py`](examples/telemetry_loop.py) | Live telemetry display with continuous speed |

---

## API Reference

### `Robot(command_address, telemetry_address)`

```python
robot = Robot("tcp://localhost:5555", "tcp://localhost:5556")
```

Both arguments are ZeroMQ TCP endpoints. The command socket uses DEALER/ROUTER, the telemetry socket uses PUB/SUB.

---

### Lifecycle

| Method | Description |
|---|---|
| `connect()` | Handshake with the sim. Raises `RobotConnectionError` on failure or timeout. |
| `disconnect()` | Gracefully disconnect and close sockets. |
| `run()` | **Non-blocking** update tick — call this in your loop. Polls sockets, updates telemetry, sends heartbeats. |
| `run_to_position()` | **Blocking** — calls `run()` in a loop until the last command completes. |
| `is_move_finished()` | Returns `True` if the last sent command has completed. |

---

### Telemetry

Call `robot.run()` first to ensure values are fresh. All angles are in **degrees**, distances in **mm**, speeds in **mm/s**.

```python
robot.run()

x   = robot.get_distance_traveled_x()   # mm
y   = robot.get_distance_traveled_y()   # mm
vx  = robot.get_velocity_x()            # mm/s
vy  = robot.get_velocity_y()            # mm/s
ang = robot.get_chassis_angle()         # degrees
```

| Method | Returns | Description |
|---|---|---|
| `get_distance_traveled_x()` | `float` | X distance traveled (mm) |
| `get_distance_traveled_y()` | `float` | Y distance traveled (mm) |
| `get_velocity_x()` | `float` | Local X velocity (mm/s) |
| `get_velocity_y()` | `float` | Local Y velocity (mm/s) |
| `get_front_angle()` | `float` | Front module angle (deg) |
| `get_chassis_angle()` | `float` | Chassis heading angle (deg) |
| `get_chassis_angular_velocity()` | `float` | Chassis angular velocity (deg/s) |
| `get_wheel_speed(wheel_index)` | `float` | Speed of wheel at index (mm/s) |
| `get_wheel_distance(wheel_index)` | `float` | Distance traveled by wheel at index (mm) |

---

### Movement Commands

All commands return immediately. Use `run_to_position()` to wait for completion, or `run()` in a loop.

**Units:** speeds in mm/s · distances in mm · angles in **degrees** · times in seconds

#### High-level (with pivot point)

```python
robot.move_by_distance(distance_mm=500, x_speed=100, y_speed=0, rotation_speed=0,
                       center_x_mm=0, center_y_mm=0, rotate_chassis=False)

robot.move_by_time(time_s=2.0, x_speed=100, y_speed=0, rotation_speed=20,
                   center_x_mm=0, center_y_mm=0, rotate_chassis=True)

robot.move_at_speed(x_speed=80, y_speed=0, rotation_speed=0,
                    center_x_mm=0, center_y_mm=0, rotate_chassis=False)

robot.move_by_angle(x_speed=0, y_speed=0, angle_deg=90, rotation_speed=50,
                    center_x_mm=0, center_y_mm=0, rotate_chassis=True)
```

#### Raw (no pivot)

```python
robot.move_by_distance_raw(distance_mm=500, x_speed=100, y_speed=0,
                            rotation_speed=0, front_rotation_speed=0)

robot.move_by_time_raw(time_s=2.0, x_speed=0, y_speed=100,
                       rotation_speed=0, front_rotation_speed=0)

robot.move_at_speed_raw(x_speed=80, y_speed=0, rotation_speed=0, front_rotation_speed=0)

robot.move_by_angle_raw(angle_deg=90, x_speed=0, y_speed=0,
                        rotation_speed=50, front_rotation_speed=0)
```

#### Stop

```python
robot.stop()               # stop all motors immediately
robot.clear_command_queue() # discard all queued commands on the sim
```

#### Per-motor control

```python
robot.move_at_speed_motors([100, -100, 50, -50])  # set all motors at once

robot.run_motor_for_time(motor_id=0, speed=100, time_s=2.0)
robot.run_motor_for_distance(motor_id=1, speed=80, distance_mm=500)
robot.start_motor(motor_id=0, speed=60)
robot.stop_motor(motor_id=0)
```

---

### Exceptions

| Exception | When raised |
|---|---|
| `RobotConnectionError` | `connect()` times out or receives an unexpected response |
| `RobotCommandError` | The sim returns `CMD_ERROR` for a sent command |
