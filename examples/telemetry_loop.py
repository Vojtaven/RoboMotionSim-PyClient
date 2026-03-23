"""Non-blocking loop example — set speed and print live telemetry."""

import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from robomotionsim_client import Robot

robot = Robot("tcp://localhost:5555", "tcp://localhost:5556")
robot.connect()
print("Connected to simulation")

# Start moving continuously
robot.move_at_speed_raw(x_speed=80, y_speed=0, rotation_speed=0, front_rotation_speed=0)

start = time.monotonic()
while time.monotonic() - start < 5.0:
    robot.run()

    print(
        f"pos=({robot.get_distance_traveled_x():7.1f}, {robot.get_distance_traveled_y():7.1f})  "
        f"vel=({robot.get_velocity_x():6.1f}, {robot.get_velocity_y():6.1f})  "
        f"angle={robot.get_chassis_angle():6.1f} deg",
        end="\r",
    )
    time.sleep(0.05)

print()

# Stop and disconnect
robot.stop()
robot.run_to_position()
robot.disconnect()
print("Disconnected")
