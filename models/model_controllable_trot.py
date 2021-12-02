from . import model_trot
import time

# Must extend the base class
class quadruped(model_trot.quadruped):
    def __init__(self, lay_height=3, **kwargs):
        # Initialise the base class
        super().__init__(**kwargs)

        # All variables below are custom variables that are specific to this custom quadruped. They are used to control the robot

        # State constants
        self.STATE_LYING = 0
        self.STATE_STANDING = 1
        self.STATE_TROTTING = 2
        self.STATE_PUSHUP = 3

        # Scripts can directly modify these to control the robot
        self.lay_height = lay_height
        self.state = self.STATE_LYING


    # This method is a special method that gets called once every update step
    def _on_update(self):
        current_t = time.time()
        self.t += (current_t - self.last_update_time) * self.trot_frequency
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
        lay_pos = self.get_dir("body.down") * self.lay_height
        for l in range(4):
            self.quad_controller.set_leg(l, lay_pos)

    # The behaviour for the press up down state
    def state_pu(self):
        lay_pos = self.get_dir("body.down") * self.lay_height
        push_pos = self.get_dir("body.down") * 8
        self.quad_controller.set_leg(0, lay_pos)
        self.quad_controller.set_leg(1, lay_pos)
        self.quad_controller.set_leg(2, push_pos)
        self.quad_controller.set_leg(3, push_pos)

    # The behaviour for the stand state
    def state_stand(self):
        # Set all legs to default height
        posses = self.calculate_still_positions()
        self.quad_controller.set_all_legs(posses)
