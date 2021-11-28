"""
This script creates an instance of trotting_robot (without a motor controller or rotation sensor attached), then benchmarks it.
"""
from models import model_trot as trotting_robot
import time

r = trotting_robot.quadruped()

r.speed = [1, 0]

t = time.time()
for i in range(1000):
    r.update()

spt = (time.time()-t)/1000
print("Without trotting algoritm running -> Updates per second:", 1/spt)

r.state = r.STATE_TROTTING
t = time.time()
for i in range(1000):
    r.update()

spt = (time.time()-t)/1000
print("With trotting algoritm running -> Updates per second:", 1/spt)
