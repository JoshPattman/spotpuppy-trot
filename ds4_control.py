# Module for using dulashock 4 controller with raspberry pi using evdev
# By Josh Pattman

from evdev import InputDevice, categorize, ecodes


# Find all the button and joystick value events that the gamepad has
def map_buttons(gamepad):
    buttons = []
    joyVals = {}
    events = []
    try:
        for e in gamepad.read():
            events.append(e)
    except BlockingIOError:
        pass

    for event in events:
        # Button Pressed
        if event.value == 1 and event.type == 1:
            buttons.append(event.code)
        # Joystick
        elif event.type == 3:
            joyVals[event.code] = event.value
    return (buttons, joyVals)


def get_default_ds4(waitForConnection=True):
    v = True
    while v:
        for i in range(10):
            try:
                tgp = InputDevice('/dev/input/event%s' % i)
                if tgp.name in ["Sony Computer Entertainment Wireless Controller", "Wireless Controller"]:
                    return tgp
            except:
                pass
        v = (False if not waitForConnection else v)
    return None


buttons = 0
joysticks = 1

standard_joystick_values = {0: 125, 1: 125, 3: 125, 4: 125}


class ds4_controller:
    def __init__(self, gamepad=None, waitForConnection=False):
        if gamepad is None:
            gamepad = get_default_ds4(waitForConnection=waitForConnection)
        if gamepad is None:
            print("Gamepad could not be found")
        self.gamepad = gamepad
        self.pressed = []
        self.old_pressed = []
        self.joystick_values = dict(standard_joystick_values)

        # The Buttons X, Circle, Triangle, and Square
        self.btn_x = 304
        self.btn_o = 305
        self.btn_t = 307
        self.btn_s = 308

        # The Buttons L/R 1-3
        self.btn_l1 = 310
        self.btn_l2 = 312
        self.btn_l3 = 317
        self.btn_r1 = 311
        self.btn_r2 = 313
        self.btn_r3 = 318

        # The Joysticks
        self.joy_throttle = 1
        self.joy_rudder = 0
        self.joy_elevator = 4
        self.joy_aileron = 3

    def refresh_inputs(self):
        inputs = map_buttons(self.gamepad)
        self.old_pressed = self.pressed
        self.pressed = inputs[buttons]
        for k in inputs[joysticks]:
            self.joystick_values[k] = inputs[1][k]

    def get_joystick(self, axis_number, normalise=True):
        val = self.joystick_values[axis_number]
        if not normalise:
            return val
        else:
            return (2 * val / 255) - 1

    def get_button(self, button_number):
        return button_number in self.remote.pressed

    def get_button_down(self, button_number):
        return (button_number in self.pressed) and (button_number not in self.old_pressed)

    def get_button_up(self, button_number):
        return (button_number not in self.pressed) and (button_number in self.old_pressed)
