"""Basic movement example — move forward, turn, and print telemetry."""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from robomotionsim_client import Robot

robot = Robot("tcp://localhost:5555", "tcp://localhost:5556")
robot.connect()
print("Connected to simulation")

# Move forward 500mm
print("Moving forward 500mm...")
robot.move_by_distance_raw(distance_mm=500, x_speed=100, y_speed=0,
                           rotation_speed=0, front_rotation_speed=0)
robot.run_to_position()
print(f"  Position: x={robot.get_distance_traveled_x():.1f} y={robot.get_distance_traveled_y():.1f}")

# Turn 90 degrees
print("Turning 90 degrees...")
robot.move_by_angle_raw(angle_deg=90, x_speed=0, y_speed=0,
                        rotation_speed=50, front_rotation_speed=0)
robot.run_to_position()
print(f"  Chassis angle: {robot.get_chassis_angle():.1f} deg")

# Move forward again
print("Moving forward 300mm...")
robot.move_by_distance_raw(distance_mm=300, x_speed=100, y_speed=0,
                           rotation_speed=0, front_rotation_speed=0)
robot.run_to_position()
print(f"  Position: x={robot.get_distance_traveled_x():.1f} y={robot.get_distance_traveled_y():.1f}")

robot.disconnect()
print("Disconnected")
