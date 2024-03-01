

from comm.Serial import SerialController, DataType_Int, DataType_Float
from joystick.JoystickManager import JoystickManager
from gui.simpleGUI import SimpleGUI
import time
import math

##### Insert your robot's MAC ADDRESS here ####
## (you can get it by running your arduino and looking at the serial monitor for your flying drone) ##
ROBOT_MAC = "48:27:E2:E6:ED:24"
### Insert your SERIAL PORT here ###
## may look like "COM5" in windows or "/dev/tty.usbmodem14301" in mac  #
## look in arduino for the port that your specific transeiver is connected to  ##
## Note: make sure that your serial monitor is OFF in arduino or else you will get "access is denied" error. ##
PORT = "/dev/cu.usbmodem1101"


# For debug purposes
PRINT_JOYSTICK = False


BaseStationAddress = "" # you do not need this, just make sure your DroneMacAddress is not your base station mac address


VELOCITY_SCALING = 0.5  # Scale factor for converting trigger input to velocity
motor_offset = 0
STEERING_SCALING = 0.6  # Scale factor for converting joystick input to steering angle
DEADZONE = 0.  # Joystick deadzone
UPDATE_INTERVAL = 0.05  # Time interval for each control loop iteration in seconds
theta = 0  # Heading of the car, starts at 0 radians
AUTO = 0
desire_height = 1



if __name__ == "__main__":
    # Communication
    serial = SerialController(PORT, timeout=.1)  # 5-second timeout
    serial.manage_peer("A", ROBOT_MAC)
    serial.manage_peer("G", ROBOT_MAC)
    serial.send_preference(ROBOT_MAC, DataType_Float, "kpz", .6)
    serial.send_preference(ROBOT_MAC, DataType_Float, "kdz", .8)
    serial.send_control_params(ROBOT_MAC, (0,0,90,90, 0, 0, 0, 0, 0, 0, 0, 1, 0)) #refresh parameters
    time.sleep(.2)

    # Joystick
    joystick = JoystickManager()
    mygui = SimpleGUI()

    try:
        while True:
            # Axis input: [left_vert, left_horz,left_trigger, right_vert, right_horz, right_trigger]
            # Button inputs: [A, B, X, Y]
            axis, buttons = joystick.getJoystickInputs()


            # Map the joystick input (from -1 to 1) to the servo range (0 to 180 degrees)
            def map_joystick_to_servo(value, min_input, max_input, min_output, max_output):
                # Convert the input value within the input range to a proportion in the output range
                return (value - min_input) * (max_output - min_output) / (max_input - min_input) + min_output



            if PRINT_JOYSTICK:
                print(" ".join(["{:.1f}".format(num) for num in axis]), buttons)

            # Mapping joystick inputs to controls
            velocity = ((axis[5] + 1) / 2) * VELOCITY_SCALING
            steering_input = axis[1] * STEERING_SCALING

            # Calculate differential drive motor speeds
            # Here we assume a simple proportional control for steering.
            m1 = velocity - (steering_input / 2)
            m2 = velocity + (steering_input / 2)

            # Make sure motor speeds are within bounds [0, 1]
            m1 = max(0.025, min(1, m1))
            m2 = max(0.025, min(1, m2))

            # Calculate new heading based on steering input
            theta += steering_input * UPDATE_INTERVAL
            theta %= 2 * math.pi  # Keep theta within [0, 2*pi]

            # Servo control
            # default position for servo
            if abs(axis[3]) < DEADZONE:
                s1 = 150
                s2 = 150
            else:
                s1 = map_joystick_to_servo(-axis[3], -1, 1, 0, 180)
                s2 = map_joystick_to_servo(-axis[3], -1, 1, 0, 180)




            #s1 = s1 - motor_offset;
            ############# End CONTROL INPUTS ###############
            sensors = serial.getSensorData()


            # Toggle AUTO if button B (assuming index 1) is pressed
            if buttons[1]:  # Assuming "B" button is at index 1
                AUTO = 1 - AUTO
                print(f"AUTO mode toggled to {AUTO}")
                time.sleep(UPDATE_INTERVAL)  # Debounce delay

            # Increase desire_height if button Y (assuming index 3) is pressed
            if buttons[3]:  # Assuming "Y" button is at index 3
                desire_height += 1  # Adjust the increment as needed
                print(f"Desired height increased to {desire_height}")
                time.sleep(UPDATE_INTERVAL)  # Debounce delay

            # Decrease desire_height if button A (assuming index 0) is pressed
            if buttons[0]:  # Assuming "A" button is at index 0
                desire_height -= 1  # Adjust the decrement as needed
                if desire_height < 0: desire_height = 0  # Prevent negative height
                print(f"Desired height decreased to {desire_height}")
                time.sleep(UPDATE_INTERVAL)

            print(sensors)
            if (sensors):
                mygui.update(
                    cur_yaw=0,
                    des_yaw=-0,
                    cur_height=sensors[0],
                    des_height=desire_height,
                    battery=0,
                    distance=0,
                    connection_status=True,
                )
            # Send through serial port

            # Axis input: [left_vert, left_horz,left_trigger, right_vert, right_horz, right_trigger]
            # Button inputs: [A, B, X, Y]


            # Debug output
            print(f"s1: {s1:.2f}, s2: {s2:.2f}, m1: {m1:.2f}, m2: {m2:.2f}, theta: {math.degrees(theta):.2f}")
            serial.send_control_params(ROBOT_MAC, (m1, m2, s1, s2, 0, desire_height, 0, 0, 0, 0, 0, 0, 0))
            time.sleep(UPDATE_INTERVAL)
            
    except KeyboardInterrupt:
        print("Stopping!")
    # Send zero input
    serial.send_control_params(ROBOT_MAC, (0, 0, 90, 90,0, -100, 0, 0, 0, 0, 0, 0, 0))
    serial.close()
