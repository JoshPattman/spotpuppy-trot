from spotpuppy.models import quadruped_base
import math, time

class quadruped (quadruped_base.quadruped):
    def __init__(self, rotation_sensor=None, servo_controller=None, bone_length=6, body_dims=[10, 10], fall_rotation_limit=0, lay_height=3):
        quadruped_base.quadruped.__init__(self, rotation_sensor=rotation_sensor,servo_controller=servo_controller,bone_length=bone_length,body_dims=body_dims,fall_rotation_limit=fall_rotation_limit)
        self.STATE_LYING = 0
        self.STATE_STANDING = 1
        self.STATE_TROTTING = 2

        self.lay_height = lay_height
        self.step_height = 4

        self.last_update_time = time.time()

        self.state = self.STATE_LYING
        self.frequency = 1
        self.t = 0
        self.air_multiplier = 1
        self.speed = [0, 0]

    def _on_update(self):
        current_t = time.time()
        self.t += (current_t - self.last_update_time)*self.frequency
        self.last_update_time = current_t

        if self.state == self.STATE_STANDING:
            self.state_stand()
        elif self.state == self.STATE_LYING:
            self.state_lay()
        else:
            self.state_trot()

    def state_lay(self):
        # Set all legs to height of self.lay_height
        lay_pos = self.get_dir("body.down")*self.lay_height
        for l in range(4):
            self.quad_controller.set_leg(l, lay_pos)

    def state_stand(self):
        # Set all legs to default height
        posses = self._calculate_still_positions()
        self.quad_controller.set_all_legs(posses)

    def state_trot(self):
        ta = self.t%1.0
        tb = (self.t+0.5)%1.0
        v_clock = [get_foot_height(ta, s=self.air_multiplier), get_foot_height(tb, s=self.air_multiplier)]
        h_clock = [get_foot_horiz(ta, s=self.air_multiplier), get_foot_horiz(tb, s=self.air_multiplier)]
        leg_sync = [0, 1, 1, 0]
        for l in range(4):
            # Get the position of the foot on the floor under the shoulder when body is at height 8
            f_pos = self.get_vector_to_robot_center(l, "body")\
                    + (self.get_dir("global.down") * 8)\
                    - self.get_vector_to_robot_center(l, "global")
            # Calculate the vectors for step offset
            v_off = v_clock[leg_sync[l]] * self.step_height * self.get_dir("global.down") * -1
            f_off = h_clock[leg_sync[l]] * self.speed[0] * self.get_dir("global.forward")
            l_off = h_clock[leg_sync[l]] * self.speed[1] * self.get_dir("global.left")
            roll_pitch = self.get_roll_pitch()
            self.quad_controller.set_leg(l, f_pos + v_off + f_off + l_off)


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
