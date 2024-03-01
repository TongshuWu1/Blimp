
from comm.Serial import SerialController, DataType_Int, DataType_Float
# Make sure to import KeyboardManager from its location in your project
from joystick.KeyboardManager import KeyboardManager
from gui.simpleGUI import SimpleGUI
import time


##### Insert your robot's MAC ADDRESS here ####
## (you can get it by running your arduino and looking at the serial monitor for your flying drone) ##
ROBOT_MAC = "34:85:18:8D:A0:D4" # "DC:54:75:D7:B3:E8"
### Insert your SERIAL PORT here ###
## may look like "COM5" in windows or "/dev/tty.usbmodem14301" in mac  #
## look in arduino for the port that your specific transeiver is connected to  ##
## Note: make sure that your serial monitor is OFF in arduino or else you will get "access is denied" error. ##
PORT = "/dev/ttyUSB0"

# For debug purposes
PRINT_JOYSTICK = False

if __name__ == "__main__":
    # Communication
    serial = SerialController(PORT, timeout=.1)  # 5-second timeout
    serial.manage_peer("A", ROBOT_MAC)
    serial.manage_peer("G", ROBOT_MAC)
    serial.send_preference(ROBOT_MAC, DataType_Float, "kpz", .6)
    serial.send_preference(ROBOT_MAC, DataType_Float, "kdz", .8)
    serial.send_control_params(ROBOT_MAC, (0,0,90,90, 0, 0, 0, 0, 0, 0, 0, 1, 0)) #refresh parameters
    time.sleep(.2)

    # Keyboard
    keyboard = KeyboardManager()
    mygui = SimpleGUI()

    try:
        while True:
            # Axis input: [ws, ad, V^, <>, q, e] (these duo letters represent a "joystick" axis equivalent)
            # Button inputs: [Z, X, C, V] mapped to [A, B, X, Y]
            axis, buttons = keyboard.getKeyboardInputs()

            if PRINT_JOYSTICK:
                print("Axis:", " ".join(["{:.1f}".format(num) for num in axis]), "Buttons:", buttons)

            #### CONTROL INPUTS to the robot here #########
            # Example control inputs (modify as necessary for your application)
            m1 = 0.  # Motor 1: a value between 0-1
            m2 = 0.  # Motor 2: a value between 0-1
            s1 = 0.  # Servo 1: a value between 0-180
            s2 = 0.  # Servo 2: a value between 0-180
            led = axis[0]  # Example: use the vertical input from the left joystick to control an LED

            ############# End CONTROL INPUTS ###############
            sensors = serial.getSensorData()
            print(sensors)
            if (sensors):
                mygui.update(
                    cur_yaw=0,
                    des_yaw=-0,
                    cur_height=sensors[0],
                    des_height=0,
                    battery=0,
                    distance=0,
                    connection_status=True,
                )
            # Send through serial port
            serial.send_control_params(ROBOT_MAC, (m1, m2, s1, s2, led, 0, 0, 0, 0, 0, 0, 0, 0))
            time.sleep(.02)
            
    except KeyboardInterrupt:
        print("Stopping!")
        # Send zero input
    serial.send_control_params(ROBOT_MAC, (0, 0, 180, 180, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    serial.close()
