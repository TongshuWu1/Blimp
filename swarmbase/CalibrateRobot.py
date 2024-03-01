'''
Calibrate your robot!
In order to calibrate, 
    1) make sure your drone is on and your mac/port are correct
    2) run this python file
    3) unplug your robot, then replug in with a battery
    4) Listen to the motors making "music"
    5) when the music stops, your robot has finished calibrating
After calibration, your robot will function as normal, 
and you will probobly not need to run this program again.
'''



from comm.Serial import SerialController, DataType_Int, DataType_Float, DataType_Boolean
from joystick.JoystickManager import JoystickManager
import time

##### Insert your robot's MAC ADDRESS here ####
## (you can get it by running your arduino and looking at the serial monitor for your flying drone) ##
ROBOT_MAC = "48:27:E2:E6:ED:24"
### Insert your SERIAL PORT here ###
## may look like "COM5" in windows or "/dev/tty.usbmodem14301" in mac  #
## look in arduino for the port that your specific transeiver is connected to  ##
## Note: make sure that your serial monitor is OFF in arduino or else you will get "access is denied" error. ##
PORT = "/dev/cu.usbmodem11101"


# For debug purposes
PRINT_JOYSTICK = False


BaseStationAddress = "" # you do not need this, just make sure your DroneMacAddress is not your base station mac address



if __name__ == "__main__":
    # Communication
    serial = SerialController(PORT, timeout=.1)  # 5-second timeout
    serial.manage_peer("A", ROBOT_MAC)
    serial.send_preference(ROBOT_MAC, DataType_Boolean, "calibrate", True)
    serial.close()