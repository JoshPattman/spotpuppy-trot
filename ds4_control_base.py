#Simple Library for reading Dualashock 4 controllers over bluetooth on a raspi - By Josh Pattman

IS_EVDEV_INSTALLED = True

try:
    from evdev import InputDevice, categorize, ecodes
except:
    IS_EVDEV_INSTALLED = False
    print("'evdev' is not installed (pip install evdev). The controller will be a dummy")

import time

#The Buttons X, Circle, Triangle, and Square
btnX = 304
btnO = 305
btnT = 307
btnS = 308

#The Buttons L/R 1-3
L1 = 310
L2 = 312
L3 = 317
R1 = 311
R2 = 313
R3 = 318

#The Joysticks
Throttle = 1
Rudder = 0
Elevator = 4
Aileron = 3

#Make a code readable
def convertCode(c):
    if c == btnX:
        return "X"
    elif c == btnO:
        return "Circle"
    elif c == btnT:
        return "Triangle"
    elif c == btnS:
        return "Square"
    elif c == R1:
        return "R1"
    elif c == R2:
        return "R2"
    elif c == R3:
        return "R3"
    elif c == L1:
        return "L1"
    elif c == L2:
        return "L2"
    elif c == L3:
        return "L3"
    else:
        return ""

#Find all the button and joystick value events that the gamepad has
def mapButtons(gamepad):
    buttons = []
    joyVals = {}
    events = []
    try:
        for e in gamepad.read():
            events.append(e)
    except BlockingIOError:
        pass

    for event in events:
        #Button Pressed
        if event.value == 1 and event.type == 1:
            buttons.append(event.code)
        #Joystick
        elif event.type == 3:
            joyVals[event.code] = event.value
    return (buttons, joyVals)

#Find the default DS4 controller connected to th Pi
def returnDefaultGamePad():
    for i in range(10):
        try:
            tgp = InputDevice('/dev/input/event%s'%i)
            if tgp.name in ["Sony Computer Entertainment Wireless Controller", "Wireless Controller"]:
                return tgp
        except:
            pass
    return None

buttons = 0
joysticks = 1

joyVals = {0:125, 1:125, 3:125, 4:125}

#Class for easily reading a DS4 controller (can also be run on windows, but will be a dummy)
class DS4Remote:
    def __init__(self, waitForConnection = False, gamepad=None):
        if IS_EVDEV_INSTALLED:
            if gamepad == None:
                gamepad = returnDefaultGamePad()
            if gamepad == None:
                if waitForConnection:
                    print("Waiting for DS4 connection")
                    while gamepad == None:
                        gamepad = returnDefaultGamePad()
                        time.sleep(0.5)
                else:
                    print("Could not connect to DS4 controller")
            self.gamepad = gamepad
        else:
            self.gamepad = None
            print("Dummy controller created because evdev could not be found")
        self.pressed = []
        self.old_pressed = []
        self.joystickValues = dict(joyVals)
    def refreshInputs(self):
        if not self.gamepad == None:
            inputs = mapButtons(self.gamepad)
            self.old_pressed = self.pressed
            self.pressed = inputs[buttons]
            for k in inputs[joysticks]:
                self.joystickValues[k] = inputs[1][k]


#Example of use, where the thottle controls how many '-' signs are printed per line
if __name__ == "__main__":
    remote = DS4Remote(waitForConnection = True)
    print("Press X to exit")
    while not btnX in remote.pressed:
        remote.refreshInputs()
        print("-" * remote.joystickValues[Throttle])
