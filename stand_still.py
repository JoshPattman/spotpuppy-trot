from spotpuppy.models import basic_quadruped
from spotpuppy.servo.servokit_servo_controller import controller

r = basic_quadruped.quadruped(servo_controller=controller())
r.load_config_folder("robot_config.rbt")

r.update()
