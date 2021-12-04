"""
This script creates an instance of trotting_robot (without a motor controller or rotation sensor attached), then benchmarks it.
"""
from models import model_trot as trotting_robot
import time
from spotpuppy.utils.time_util import max_ups

r = trotting_robot.quadruped()
r.load_config_folder("robot_config.rbt")
t = time.time()
for i in range(1000):
    r.update()

spt = (time.time()-t)/1000
print("Uncapped -> Updates per second:", 1/spt)

m = max_ups(100)
t = time.time()
for i in range(200):
    r.update()
    m.update()

spt = (time.time()-t)/200
print("Max UPS at 100 -> Updates per second:", 1/spt)
