"""
This script creates an instance of trotting_robot with a servokit servo controller (adafruit)
It loads the robot from directory "robot_config.rbt"
It then causes the robot to follow the ds4 controller commands
"""

from peripherals import ds4_control
from models import model_controllable_trot
import time

from spotpuppy.servo.servokit_servo_controller import controller as servo_controller
from spotpuppy.rotation.arduino_rotation_sensor import sensor
from spotpuppy.utils.robot_update_thread import start_threaded_updates

ds4 = ds4_control.ds4_controller()
print("Controller connected")

r = model_controllable_trot.quadruped(servo_controller=servo_controller(), rotation_sensor=sensor())
r.load_config_folder("robot_config.rbt")
print("Loaded robot")

r.state=r.STATE_LYING
r.update()
r.rotation_sensor.calibrate()
print("Calibrated sensor")

r.trot_step_length = [3, 0]
r.trot_air_multiplier = 0.5
r.trot_frequency = 1.5
r.trot_step_height = 5
r.lean = [0, 0.5]
r.trot_tile_move_mult = 0.3

# Start the robot updating at 50 times per second
start_threaded_updates(r, 40, warn_if_low=True)

# At 20 times per second, update the controller and tell the robot to move
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
    time.sleep(0.05)
