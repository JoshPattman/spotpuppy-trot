import ds4_control

import trotting_robot
import time
from spotpuppy.servo.servokit_servo_controller import controller as servo_controller
from spotpuppy.utils import json_serialiser

ds4 = ds4_control.ds4_controller()

r = trotting_robot.quadruped(servo_controller=servo_controller())
json_serialiser.load_into_robot(r, "SP3.rbt")

r.speed = [2, 0]
r.air_multiplier = 0.5
r.frequency = 2
r.step_height = 6
r.lean = [0, -0.5]

r.state = r.STATE_LYING

while True:
    ds4.refresh_inputs()
    if ds4.get_button_down(ds4.btn_x):
        r.state = r.STATE_LYING 
    if ds4.get_button_down(ds4.btn_s):
        r.state = r.STATE_TROTTING
    if ds4.get_button_down(ds4.btn_t):
        r.state = r.STATE_STANDING
    if ds4.get_button_down(ds4.btn_o):
        r.state = r.STATE_PUSHUP
    r.speed = [-3*ds4.get_joystick(ds4.joy_throttle), -2*ds4.get_joystick(ds4.joy_aileron)]
    r.update()
