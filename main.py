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
    open(servol) 

    # --- Connect to Robot ---
    rtde_c = RTDEControl(ROBOT_IP)
    rtde_r = RTDEReceive(ROBOT_IP)
    print("Successfully connected to robot.")

        # --- Get Initial Position (Optional but good practice) ---
    initial_pose = rtde_r.getActualTCPPose()
    print(f"Initial TCP Pose: {initial_pose}")

    image = take_picture()
    print("Image captured from camera.")
    show_image(image)
    save_image(image, 'captured_image.jpg')

    # --- Move to Initial Position ---
    target_x1 = -0.5
    target_y1 = -0.15

    target_x2 = -0.5
    target_y2 = 0.15

    target_pose1 = [target_x1, target_y1, Z_HEIGHT] + FIXED_ORIENTATION
    safe_pose1 = safe_pos(target_pose1)

    target_pose2 = [target_x2, target_y2, Z_HEIGHT] + FIXED_ORIENTATION
    safe_pose2 = safe_pos(target_pose2)


    print(f"Moving to Pose 1: {target_pose1}")
    rtde_c.moveL(safe_pose1, SPEED, ACCELERATION) 
    rtde_c.moveL(target_pose1, SPEED, ACCELERATION)
    stop_move(rtde_c)
    print("Reached Pose 1.")
    rtde_c.moveL(safe_pose1, SPEED, ACCELERATION) 


    print(f"Moving to Pose 2: {target_pose2}")
    rtde_c.moveL(safe_pose2, SPEED, ACCELERATION) 
    rtde_c.moveL(target_pose2, SPEED, ACCELERATION)
    stop_move(rtde_c)
    print("Reached Pose 2.")
    rtde_c.moveL(safe_pose2, SPEED, ACCELERATION)  


    time.sleep(1)


except Exception as e:
    print(f"An error occurred: {e}")

finally:
    # --- Cleanup ---
    print("Cleaning up resources...")
    stop_image()
    open(servol) 
    stop(servol)
    rtde_c.disconnect()
    print("Program finished.")

