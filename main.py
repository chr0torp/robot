import time
import math
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

from robot_utils import *
from move_utils import *
from camara_utils import *
from detect_utils import *


# --- Configuration ---
ROBOT_IP = "192.168.1.102"  # Replace with your robot's actual IP address
Z_HEIGHT = 0.31             # Desired constant Z height (in meters)
SPEED = 0.3                # TCP speed (m/s)
ACCELERATION = 0.01         # TCP acceleration (m/s^2)

# Define the fixed orientation (tool's Z-axis points down along base -Z)
# Rotation of pi radians (180 degrees) around the base X-axis
FIXED_ORIENTATION = [math.pi, 0, 0]
# --- End Configuration ---

try:
    servol = start_servo()

    # --- Connect to Robot ---
    rtde_c = RTDEControl(ROBOT_IP)
    rtde_r = RTDEReceive(ROBOT_IP)
    print("Successfully connected to robot.")

        # --- Get Initial Position (Optional but good practice) ---
    initial_pose = rtde_r.getActualTCPPose()
    print(f"Initial TCP Pose: {initial_pose}")

    picam2 = start_camera()
    show_camera_feed(picam2)  # This will run until 'q' is pressed


    quit_key()


except Exception as e:
    print(f"An error occurred: {e}")

finally:
    # --- Disconnect ---
    stop(servol)
    stop_camera(picam2)
    if 'rtde_c' in locals() and rtde_c.isConnected():
        print("Stopping script and disconnecting.")
        rtde_c.stopScript()
    print("Program finished.")

