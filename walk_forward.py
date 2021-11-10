"""
This script creates an instance of trotting_robot with a servokit servo controller (adafruit)
It loads the robot from directory "SP3.rbt"
It then causes the robot to walk forwards at a constant speed forever
"""

import trotting_robot
import time
from spotpuppy.servo.servokit_servo_controller import controller
from spotpuppy.utils import json_serialiser
from spotpuppy.rotation.arduino_rotation_sensor import sensor

r = trotting_robot.quadruped(servo_controller=controller(), rotation_sensor=sensor())
json_serialiser.load_into_robot(r, "SP3.rbt")
r.state=r.STATE_LYING
r.update()
r.rotation_sensor.calibrate()

r.speed = [3, 0]
r.air_multiplier = 0.5
r.frequency=2
r.step_height = 5
#r.lay_height = 8

r.state = r.STATE_TROTTING

while True:
    r.update()
