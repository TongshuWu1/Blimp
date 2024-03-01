
import pygame, time
import numpy as np




DroneMacAddress = "DC:54:75:D7:B3:E8" #insert your blimp's mac address here (you can get it by running your arduino and looking at the serial monitor for your flying drone)
BaseStationAddress = "" # you do not need this, just make sure your DroneMacAddress is not your base station mac address
port = "COM6" # may look like "COM5" or "/dev/tty.usbmodem14301", look in arduino for the port that your specific transeiver is connected to
#note: make sure that your serial monitor is OFF on your base station in arduino or else you will get "access is denied" error

class JoystickManager:
    def __init__(self):

        # Initialize Pygame for joystick handling
        pygame.init()
        pygame.joystick.init()  # Initialize the Joystick subsystem

        # Assuming there's at least one joystick connected
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)  # Initialize the first joystick
            self.joystick.init()
        else:
            print("No joystick detected!")
            self.joystick = None

    def updateJoy(self):
        pygame.event.pump()  # Process event queue for the joystick
        # time.sleep(.1)  # TODO: is it necesary?

    def getJoysticks(self):
        if self.joystick:
            # Example mapping (adjust based on your joystick)
            right_vert = self.joystick.get_axis(4)  # Right joystick vertical3
            right_horz = self.joystick.get_axis(3)  # Right joystick horizontal2
            left_vert = self.joystick.get_axis(1)  # Left joystick vertical
            left_horz = self.joystick.get_axis(0)  # Left joystick horizontal
            right_trigger = self.joystick.get_axis(5)  # Right trigger
            left_trigger = self.joystick.get_axis(2)  # Left trigger
            return [left_vert, left_horz, right_vert, right_horz, left_trigger, right_trigger]
        else:
            return [0, 0, 0, 0, 0, 0]

    def getButtons(self):
        if self.joystick:
            # Example button mapping (adjust based on your joystick)
            a = self.joystick.get_button(0)
            b = self.joystick.get_button(1)
            x = self.joystick.get_button(2)
            y = self.joystick.get_button(3)
            return [a, b, x, y]
        else:
            return [0, 0, 0, 0]


    def getJoystickInputs(self):
        self.updateJoy()
        return self.getJoysticks(), self.getButtons()



if __name__ == "__main__":


    joystick = JoystickManager()

    try:
        while True:
            axis, buttons = joystick.getJoystickInputs()
            # Print the output
            print(" ".join(["{:.1f}".format(num) for num in axis]), buttons)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Stopping!")