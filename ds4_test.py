import ds4_control

import model_controllable_trot
import time
from spotpuppy.servo.servokit_servo_controller import controller as servo_controller
from spotpuppy.utils import json_serialiser
from spotpuppy.rotation.arduino_rotation_sensor import sensor

ds4 = ds4_control.ds4_controller()

r = model_controllable_trot.quadruped(servo_controller=servo_controller(), rotation_sensor=sensor())
json_serialiser.load_into_robot(r, "SP3.rbt")

r.state=r.STATE_LYING
r.update()
r.rotation_sensor.calibrate()

r.trot_step_length = [3, 0]
r.trot_air_multiplier = 0.5
r.trot_frequency = 2
r.trot_step_height = 5
r.lean = [0, 0.5]

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
    r.trot_step_length = [-3*ds4.get_joystick(ds4.joy_throttle), -2*ds4.get_joystick(ds4.joy_aileron)]
    r.trot_turn = 3*ds4.get_joystick(ds4.joy_rudder)
    r.update()
