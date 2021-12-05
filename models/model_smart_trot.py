from spotpuppy.models import quadruped_base
from spotpuppy.utils.pid_control import pid_controller
import math, time
import numpy as np


# Must extend the base class
class quadruped(quadruped_base.quadruped):
    def __init__(self, **kwargs):
        # Initialise the base class
        super().__init__(**kwargs)
        # This stores the last time that the robot updated
        self.last_update_time = time.time()
        # This is the timer for the robot, it goes up by self.frequency every second
        self.t = 0
        self.trot_frequency = 3
        self.air_mult = 1
        self.pushup_pids = [pid_controller(0.1, 0, 0), pid_controller(0.1, 0, 0)]
        self.trot_speed = np.array([0, 0])
        self.tilt_walk_mult = 1
        self.step_height = 5

    # This method is a special method that gets called once every update step
    def _on_update(self):
        current_t = time.time()
        self.t += (current_t - self.last_update_time) * self.trot_frequency
        self.last_update_time = current_t
        self.state_trot()

    # Set this value from: -1 to 1
    # where 0 (no movement), 1 (absolute fastest the quad can go, but it will probs fall over)
    def set_trot_speed(self, forwards, left):
        self.pushup_pids[0].set_target(forwards*self.fall_rotation_limit)
        self.pushup_pids[1].set_target(left*self.fall_rotation_limit)

    def state_trot(self):
        # State
        roll_pitch = self.get_roll_pitch()

        # PID pushup
        pushup = [0,0]
        pushup[0] = self.pushup_pids[0].update(roll_pitch[0])
        pushup[1] = self.pushup_pids[1].update(roll_pitch[1])
        pushup_legs = [-pushup[0] + pushup[1], pushup[0] + pushup[1], -pushup[0] - pushup[1], pushup[0] - pushup[1]]

        for leg in range(4):
            # Footprint
            footprint = self.get_vector_to_robot_center(leg, "body") \
                        + (self.get_dir("global.down") * 11) \
                        - self.get_vector_to_robot_center(leg, "global")

            # Tilt walk
            t_mult = (1 / 45) * self.tilt_walk_mult
            self.trot_speed[0], self.trot_speed[1] = roll_pitch[1] * t_mult, roll_pitch[0] * t_mult

            # Gait
            trot_length = (self.trot_speed / self.trot_frequency) / self.air_mult
            gait_pos_raw_horiz, gait_pos_raw_vert = get_gait_pos(self.t, s=self.air_mult)
            gait_forwards = gait_pos_raw_horiz * trot_length[0] * self.get_dir("global.forward")
            gait_left = gait_pos_raw_horiz * trot_length[1] * self.get_dir("global.left")

            # If we are pushing up, only push up at the bottom of the step
            # If the pushup for this side is negative (leg should be closer to body)
            step_min = -pushup_legs[leg]
            step_max = self.step_height if pushup_legs[leg] > 0 else self.step_height - pushup_legs[leg]
            gait_vertical_transformed = map_range(gait_pos_raw_vert, 0, 1, step_min, step_max)
            gait_vertical = gait_vertical_transformed * self.get_dir("global.down") * -1
            gait = gait_forwards + gait_left + gait_vertical

            # Merging
            self.quad_controller.set_leg(leg, footprint + gait)


# GAIT
# t is the timer, s is a value from 0-1 which denotes the length of time that the foot is in the air
# at s = 1, as soon as one foot is place the other comes up, at s = 0.5, half the time both feet are on the ground
def get_gait_pos(t, s=1):
    return get_foot_horiz(t, s=s), get_foot_height(t, s=s)


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
    return ((x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min
