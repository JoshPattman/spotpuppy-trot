from models import model_smart_trot
import time
from spotpuppy.servo.servokit_servo_controller import controller
from spotpuppy.utils import json_serialiser
from spotpuppy.rotation.arduino_rotation_sensor import sensor
from spotpuppy.utils.robot_update_thread import start_threaded_updates

s = sensor()
s.inverse_x = True
s.inverse_z = True

r = model_smart_trot.quadruped(servo_controller=controller(), rotation_sensor=s)
json_serialiser.load_into_robot(r, "robot_config.rbt")

r.rotation_sensor.calibrate()

r.trot_freqency = 2
r.air_mult = 0.5


start_threaded_updates(r, 40, warn_if_low=True)
while True:
    # This would be where control logic goes, does not have to run at a fixed timestep
    time.sleep(1)
