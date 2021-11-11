"""
This module (not to be directly run) contains code for a simple quadrupedal robot that can trot
It is not bound to specific hardware
"""

from spotpuppy.models import quadruped_base
from spotpuppy.utils.pid_control import pid_controller
import math, time


# Must extend the base class
class quadruped(quadruped_base.quadruped):
    def __init__(self, rotation_sensor=None, servo_controller=None, bone_length=6, body_dims=[10, 10],
                 fall_rotation_limit=0, lay_height=3):
        # Initialise the base class
        quadruped_base.quadruped.__init__(self, rotation_sensor=rotation_sensor, servo_controller=servo_controller,
                                          bone_length=bone_length, body_dims=body_dims,
                                          fall_rotation_limit=fall_rotation_limit)

        # All variables below are custom variables that are specific to this custom quadruped. They are used to control the robot

        # Scripts can directly modify these to control the robot
        # The step height of the robot
        self.trot_step_height = 4
        # The frequency of steps (freq=1 means a leg will complete full cycle in 1 second)
        self.trot_frequency = 1
        # The multiplier of the amount of time that each leg spends in the air (between 0-1)
        # Lower values tend to be more stable
        self.trot_air_multiplier = 1
        # The step length for forwards and left (leaving at zero means robot will trot in place)
        self.trot_step_length = [0, 0]
        # The control value for the trot gait turning
        self.trot_turn = 0
        self.lean = [0, 0]

        # These are the pid controllers for the leg push values
        self._trot_push_pids = [pid_controller(0.1, 0, 0), pid_controller(0.1, 0, 0)]

        # These are the pid controllers for pid pushup

        # This stores the last time that the robot updated
        self.last_update_time = time.time()
        # This is the timer for the robot, it goes up by self.frequency every second
        self.t = 0

    # This method is a special method that gets called once every update step 
    def _on_update(self):
        current_t = time.time()
        self.t += (current_t - self.last_update_time) * self.trot_frequency
        self.last_update_time = current_t
        self.state_trot()

    # The behaviour for the trot state
    def state_trot(self):
        # ta and tb are clocks with values from 0-1 but tb is offset by half a cycle
        ta = self.t % 1.0
        tb = (self.t + 0.5) % 1.0
        # v_clock contains the foot heights (from 0-1) for ta and tb
        v_clock = [get_foot_height(ta, s=self.trot_air_multiplier), get_foot_height(tb, s=self.trot_air_multiplier)]
        # h_clock contains foot horizontal offsets (from 0-1) for ta and tb
        h_clock = [get_foot_horiz(ta, s=self.trot_air_multiplier), get_foot_horiz(tb, s=self.trot_air_multiplier)]
        # leg_sync says which legs use which clock (ta or tb)
        leg_sync = [0, 1, 1, 0]
        leg_turn_sync = [-1, -1, 1, 1]
        roll_pitch = self.get_roll_pitch()
        push_pids = [self._trot_push_pids[0].update(roll_pitch[1]), self._trot_push_pids[1].update(roll_pitch[0])]
        push_legs = [
            (push_pids[0] - push_pids[1]),
            (push_pids[0] + push_pids[1]),
            (-push_pids[0] - push_pids[1]),
            (-push_pids[0] + push_pids[1])
        ]
        for l in range(4):
            # Get the position of the foot on the floor under the shoulder when body is at height 8
            f_pos = self.get_vector_to_robot_center(l, "body") \
                    + (self.get_dir("global.down") * 8) \
                    - self.get_vector_to_robot_center(l, "global")
            # Calculate the vectors for step offset
            # Vertical
            v_off = 0
            if push_legs[l] > 0:
                v_off = map_range(v_clock[leg_sync[l]], 0, 1, -push_legs[l], self.trot_step_height)\
                        * self.trot_step_height * -1 * self.get_dir("global.down")
            else:
                v_off = map_range(v_clock[leg_sync[l]], 0, 1, -push_legs[l], self.trot_step_height-push_legs[l]) \
                        * self.trot_step_height * -1 * self.get_dir("global.down")
            # Forwards
            f_off = h_clock[leg_sync[l]] * self.trot_step_length[0] * self.get_dir("global.forward")
            # Left
            l_off = h_clock[leg_sync[l]] * self.trot_step_length[1] * self.get_dir("global.left")
            # Right
            r_off = h_clock[leg_sync[l]] * self.trot_turn * self.get_dir("global.left") * leg_turn_sync[l]
            # Lean
            lean_offset = (self.get_dir("global.forward") * self.lean[0]) + (self.get_dir("global.left") * self.lean[1])
            self.quad_controller.set_leg(l, f_pos + v_off + f_off + l_off + r_off + lean_offset)


# t is the timer, s is a value from 0-1 which denotes the length of time that the foot is in the air
# at s = 1, as soon as one foot is place the other comes up, at s = 0.5, half the time both feet are on the ground
def get_foot_height(t, s=1):
    if t < 0.5 * s:
        return math.sin(t * 3.14 * 2 / s)
    return 0


def get_foot_horiz(t, s=1):
    if t < 0.5 * s:
        return from_0to1_to_m1to1(2 * t / s)
    m = 2 / (s - 2)
    return from_0to1_to_m1to1(m * t - m)


def from_0to1_to_m1to1(x):
    #return (x - 0.5) * 2
    return map_range(x, 0, 1, -1, 1)


def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
