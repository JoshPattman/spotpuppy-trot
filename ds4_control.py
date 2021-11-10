# By Josh Pattman

import ds4_control_base as pi_ds4_control_base

# ************************************************DS4 on a pi constants************************************************
# The Buttons X, Circle, Triangle, and Square
DS4btnX = pi_ds4_control_base.btnX
DS4btnO = pi_ds4_control_base.btnO
DS4btnT = pi_ds4_control_base.btnT
DS4btnS = pi_ds4_control_base.btnS

# The Buttons L/R 1-3
DS4L1 = pi_ds4_control_base.L1
DS4L2 = pi_ds4_control_base.L2
DS4L3 = pi_ds4_control_base.L3
DS4R1 = pi_ds4_control_base.R1
DS4R2 = pi_ds4_control_base.R2
DS4R3 = pi_ds4_control_base.R3

# The Joysticks
DS4Throttle = pi_ds4_control_base.Throttle
DS4Rudder = pi_ds4_control_base.Rudder
DS4Elevator = pi_ds4_control_base.Elevator
DS4Aileron = pi_ds4_control_base.Aileron

BUTTON_CURRENTLY_PRESSED = 0
BUTTON_JUST_PRESSED = 1
BUTTON_NOT_PRESSED = 2


class pi_ds4_controller:
    def __init__(self, device='DEFAULT'):
        self.USABLE = pi_ds4_control_base.IS_EVDEV_INSTALLED
        if self.USABLE:
            if device == 'DEFAULT':
                self.gamepad = pi_ds4_control_base.returnDefaultGamePad()
            self.remote = pi_ds4_control_base.DS4Remote(
                gamepad=self.gamepad, waitForConnection=True)
        else:
            print(
                "Error. This controller is not useable as evdev is not installed. You must be on a pi")

    def refresh(self):
        self.remote.refreshInputs()

    def get_axis(self, axisNumber, normalise=False):
        val = self.remote.joystickValues[axisNumber]
        if not normalise:
            return val
        else:
            return (2 * val / 255) - 1

    def get_button(self, buttonNumber):
        is_pressed = buttonNumber in self.remote.pressed
        if not is_pressed:
            return BUTTON_NOT_PRESSED
        was_pressed = buttonNumber in self.remote.old_pressed
        return BUTTON_JUST_PRESSED if not was_pressed else BUTTON_CURRENTLY_PRESSED
