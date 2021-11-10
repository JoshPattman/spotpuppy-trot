import ds4_control

import trotting_robot
import time
from spotpuppy.servo.servokit_servo_controller import controller
from spotpuppy.utils import json_serialiser

ds4controller = ds4_control.pi_ds4_controller()

r = trotting_robot.quadruped(servo_controller=controller())
json_serialiser.load_into_robot(r, "SP3.rbt")

r.speed = [2, 0]
r.air_multiplier = 0.5
r.frequency=2
r.step_height = 6
r.lean = [0, -0.5]

r.state = r.STATE_LYING

while True:
    ds4controller.refresh()
    if ds4controller.get_button(ds4_control.DS4btnX) == ds4_control.BUTTON_JUST_PRESSED:
        r.state = r.STATE_LYING 
    if ds4controller.get_button(ds4_control.DS4btnS) == ds4_control.BUTTON_JUST_PRESSED:
        r.state = r.STATE_TROTTING
    if ds4controller.get_button(ds4_control.DS4btnT) == ds4_control.BUTTON_JUST_PRESSED:
        r.state = r.STATE_STANDING
    if ds4controller.get_button(ds4_control.DS4btnO) == ds4_control.BUTTON_JUST_PRESSED:
        r.state = r.STATE_PUSHUP
    r.speed = [-3*ds4controller.get_axis(ds4_control.DS4Throttle, normalise=True), -2*ds4controller.get_axis(ds4_control.DS4Rudder, normalise=True)]
    r.update()
