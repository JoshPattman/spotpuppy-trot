"""
This module (not to be directly run) contains code for a simple quadrupedal robot that can trot, stand, and lay
It is not bound to specific hardware
"""

from spotpuppy.models import quadruped_base
import math, time

# Must extend the base class
class quadruped (quadruped_base.quadruped):
    def __init__(self, rotation_sensor=None, servo_controller=None, bone_length=6, body_dims=[10, 10], fall_rotation_limit=0, lay_height=3):
        # Initialise the base class
        quadruped_base.quadruped.__init__(self, rotation_sensor=rotation_sensor,servo_controller=servo_controller,bone_length=bone_length,body_dims=body_dims,fall_rotation_limit=fall_rotation_limit)
        
        # All variables below are custom variables that are specific to this custom quadruped. They are used to control the robot
        
        # State constants
        self.STATE_LYING = 0
        self.STATE_STANDING = 1
        self.STATE_TROTTING = 2
        self.STATE_PUSHUP = 3

        # Scripts can directly modify these to control the robot
        self.lay_height = lay_height
        self.step_height = 4
        self.use_dynamic_balancing = False
        self.state = self.STATE_LYING
        self.frequency = 1
        self.air_multiplier = 1
        self.speed = [0, 0]
        self.lean = [0,0]

        # This stores the last time that the robot updated
        self.last_update_time = time.time()
        # This is the timer for the robto, it goes up by self.frequency every second
        self.t = 0

    # This method is a special method that gets called once every update step 
    def _on_update(self):
        current_t = time.time()
        self.t += (current_t - self.last_update_time)*self.frequency
        self.last_update_time = current_t
        if self.state == self.STATE_STANDING:
            self.state_stand()
        elif self.state == self.STATE_LYING:
            self.state_lay()
        elif self.state == self.STATE_PUSHUP:
            self.state_pu()
        else:
            self.state_trot()
    

    # The behaviour for the lying down state
    def state_lay(self):
        # Set all legs to height of self.lay_height
        lay_pos = self.get_dir("global.down")*self.lay_height
        for l in range(4):
            self.quad_controller.set_leg(l, lay_pos)

    # The behaviour for the press up down state
    def state_pu(self):
        lay_pos = self.get_dir("body.down")*self.lay_height
        push_pos = self.get_dir("body.down")*8
        self.quad_controller.set_leg(0, lay_pos)
        self.quad_controller.set_leg(1, lay_pos)
        self.quad_controller.set_leg(2, push_pos)
        self.quad_controller.set_leg(3, push_pos)

    # The behaviour for the stand state
    def state_stand(self):
        # Set all legs to default height
        posses = self._calculate_still_positions()
        self.quad_controller.set_all_legs(posses)

    # The behaviour for the trot state
    def state_trot(self):
        # ta and tb are clocks with values from 0-1 but tb is offset by half a cycle
        ta = self.t%1.0
        tb = (self.t+0.5)%1.0
        # v_clock contains the foot heights (from 0-1) for ta and tb
        v_clock = [get_foot_height(ta, s=self.air_multiplier), get_foot_height(tb, s=self.air_multiplier)]
        # h_clock contains foot horizontal offsets (from 0-1) for ta and tb
        h_clock = [get_foot_horiz(ta, s=self.air_multiplier), get_foot_horiz(tb, s=self.air_multiplier)]
        # leg_sync says which legs use which clock (ta or tb)
        leg_sync = [0, 1, 1, 0]
        roll_pitch = self.get_roll_pitch()
        r_p_legs = [
                (-roll_pitch[1]+roll_pitch[0])/90,
                (-roll_pitch[1]-roll_pitch[0])/90,
                (roll_pitch[1]+roll_pitch[0])/90,
                (roll_pitch[1]-roll_pitch[0])/90
                ]
        for l in range(4):
            # Get the position of the foot on the floor under the shoulder when body is at height 8
            f_pos = self.get_vector_to_robot_center(l, "body")\
                    + (self.get_dir("global.down") * 8)\
                    - self.get_vector_to_robot_center(l, "global")
            # Calculate the vectors for step offset
            v_off = v_clock[leg_sync[l]] * self.step_height * self.get_dir("global.down") * -1
            f_off = h_clock[leg_sync[l]] * self.speed[0] * self.get_dir("global.forward")
            l_off = h_clock[leg_sync[l]] * self.speed[1] * self.get_dir("global.left")
            lean = (self.get_dir("body.forward")*self.lean[0]) + (self.get_dir("body.left")*self.lean[1])
            pushup_offset = self.get_dir("body.down") * r_p_legs[l] * 5
            self.quad_controller.set_leg(l, f_pos + v_off + f_off + l_off + pushup_offset)
            #self.quad_controller.set_leg(l, f_pos)


# t is the timer, s is a value from 0-1 which denotes the length of time that the foot is in the air
# at s = 1, as soon as one foot is place the other comes up, at s = 0.5, half the time both feet are on the ground
def get_foot_height(t, s=1):
    if t < 0.5*s:
        return math.sin(t*3.14*2/s)
    return 0


def get_foot_horiz(t, s=1):
    if t < 0.5*s:
        return ztoTomto(2*t/s)
    m = 2 / (s - 2)
    return ztoTomto(m*t - m)

def ztoTomto(x):
    return (x-0.5)*2
