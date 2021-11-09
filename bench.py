import trotting_robot
import time

r = trotting_robot.quadruped()

r.speed = [1, 0]

t = time.time()
for i in range(1000):
    r.update()

spt = (time.time()-t)/1000
print("Without trotting: SPT:", spt, ", TPS:", 1/spt)

r.state = r.STATE_TROTTING
t = time.time()
for i in range(1000):
    r.update()

spt = (time.time()-t)/1000
print("With trotting: SPT:", spt, ", TPS:", 1/spt)