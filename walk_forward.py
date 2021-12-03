"""
This script creates an instance of trotting_robot with a servokit servo controller (adafruit)
It loads the robot from directory "robot_config.rbt"
It then causes the robot to walk forwards at a constant speed forever
"""

from models import model_trot as trotting_robot
import time
from spotpuppy.servo.servokit_servo_controller import controller
from spotpuppy.utils import json_serialiser
from spotpuppy.rotation.arduino_rotation_sensor import sensor
from spotpuppy.utils.robot_update_thread import start_threaded_updates

s = sensor()
s.inverse_x = True
s.inverse_z = True

r = trotting_robot.quadruped(servo_controller=controller(), rotation_sensor=s)
json_serialiser.load_into_robot(r, "robot_config.rbt")

r.rotation_sensor.calibrate()

r.trot_step_length = [3, 0]
r.trot_air_multiplier = 0.8
r.trot_frequency = 2
r.trot_step_height = 5


start_threaded_updates(r, 40, warn_if_low=True)
while True:
    # This would be where control logic goes, does not have to run at a fixed timestep
    time.sleep(1)
