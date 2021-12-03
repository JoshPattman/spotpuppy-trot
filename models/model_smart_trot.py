
from spotpuppy.models import quadruped_base
from spotpuppy.utils.pid_control import pid_controller
import math, time


# Must extend the base class
class quadruped(quadruped_base.quadruped):
    def __init__(self, **kwargs):
        # Initialise the base class
        super().__init__(**kwargs)
        # This stores the last time that the robot updated
        self.last_update_time = time.time()
        # This is the timer for the robot, it goes up by self.frequency every second
        self.t = 0
        self.trot_freqency = 3

    # This method is a special method that gets called once every update step
    def _on_update(self):
        current_t = time.time()
        self.t += (current_t - self.last_update_time) * self.trot_frequency
        self.last_update_time = current_t
        self.state_trot()

    def state_trot(self):
        for leg in range(4):
            footprint = self.get_vector_to_robot_center(leg, "body") \
                    + (self.get_dir("global.down") * 11) \
                    - self.get_vector_to_robot_center(leg, "global")
            self.quad_controller.set_leg(leg, footprint)

# GAIT
# t is the timer, s is a value from 0-1 which denotes the length of time that the foot is in the air
# at s = 1, as soon as one foot is place the other comes up, at s = 0.5, half the time both feet are on the ground
def get_gait_pos(t, s=1):
    return get_foot_horiz(t, s=s) get_foot_height()

def get_foot_height(t, s=1):
    if t < 0.5 * s:
        return math.sin(t * 3.14 * 2 / s)
    return 0

def get_foot_horiz(t, s=1):
    if t < 0.5 * s:
        return map_range(2 * t / s, 0, 1, -1, 1)
    m = 2 / (s - 2)
    return map_range(m * t - m, 0, 1, -1, 1)


def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min