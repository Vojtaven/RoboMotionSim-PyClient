"""Per-motor control example — spin individual motors."""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from robomotionsim_client import Robot

robot = Robot("tcp://localhost:5555", "tcp://localhost:5556")
robot.connect()
print("Connected to simulation")

# Run motor 0 forward for 2 seconds
print("Running motor 0 for 2s...")
robot.run_motor_for_time(motor_id=0, speed=100, time_s=2.0)
robot.run_to_position()

# Run motor 1 for 500mm
print("Running motor 1 for 500mm...")
robot.run_motor_for_distance(motor_id=1, speed=80, distance_mm=500)
robot.run_to_position()

# Set all motors at once
print("Setting all motors to different speeds...")
robot.move_at_speed_motors([100, -100, 50, -50])

# Let it run for a bit, printing wheel speeds
import time
start = time.monotonic()
while time.monotonic() - start < 3.0:
    robot.run()
    time.sleep(0.05)

# Stop everything
robot.stop()
robot.run_to_position()

robot.disconnect()
print("Disconnected")
