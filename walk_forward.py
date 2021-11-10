"""
This script creates an instance of trotting_robot with a servokit servo controller (adafruit)
It loads the robot from directory "SP3.rbt"
It then causes the robot to walk forwards at a constant speed forever
"""

import trotting_robot
import time
from spotpuppy.servo.servokit_servo_controller import controller
from spotpuppy.utils import json_serialiser

r = trotting_robot.quadruped(servo_controller=controller())
json_serialiser.load_into_robot(r, "SP3.rbt")

r.speed = [5, 0]
r.air_multiplier = 0.5
r.frequency=2
r.step_height = 6

r.state = r.STATE_TROTTING

while True:
    r.update()