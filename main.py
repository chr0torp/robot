import time
import math
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

from robot_utils import *
from servo_utils import *
from camara_utils import *
from detect_utils import *

middle_video = 1100

# --- Configuration ---
ROBOT_IP = "192.168.1.102"  # Replace with your robot's actual IP address
# Z_HEIGHT = 0.31             # Desired constant Z height (in meters)
Z_HEIGHT = 0.36
SPEED = 0.3  
FAST_SPEED = 0.5              # TCP speed (m/s)
ACCELERATION = 0.01         # TCP acceleration (m/s^2)
FAST_ACCELERATION = 0.05  # TCP acceleration for fast moves (m/s^2)

# Define the fixed orientation (tool's Z-axis points down along base -Z)
# Rotation of pi radians (180 degrees) around the base X-axis
FIXED_ORIENTATION = [math.pi, 0, 0]
# --- End Configuration ---

try:
    # servol = start_servo()
    # open(servol)

    # --- Connect to Robot ---
    rtde_c = RTDEControl(ROBOT_IP)
    rtde_r = RTDEReceive(ROBOT_IP)
    print("Successfully connected to robot.")

    initial_pose = rtde_r.getActualTCPPose()
    print(f"Initial TCP Pose: {initial_pose}")

    # image = take_picture()
    # print("Image captured from camera.")
    # show_image(image)
    # save_image(image, 'captured_image1.jpg')

    # --- Move to Initial Position ---
    target_x0 = -0.5
    target_y0 = 0.0
    target_z0 = 0.60

    target_x1 = -0.5
    target_y1 = -0.15

    target_x2 = -0.5
    target_y2 = 0.15

    target_pose0 = [target_x0, target_y0, target_z0] + FIXED_ORIENTATION

    target_pose1 = [target_x1, target_y1, Z_HEIGHT] + FIXED_ORIENTATION
    safe_pose1 = safe_pos(target_pose1)


    print(f"Moving to initial position: {target_pose0}")
    rtde_c.moveL(target_pose0, FAST_SPEED, FAST_ACCELERATION)
    stop_move(rtde_c)
    print("Reached initial position.")


    print(f"Moving to Pose 1: {target_pose1}")
    rtde_c.moveL(safe_pose1, FAST_SPEED, FAST_ACCELERATION) 
    rtde_c.moveL(target_pose1, SPEED, ACCELERATION)
    stop_move(rtde_c)
    print("Reached Pose 1.")

    center_height = False

    while not center_height:
        image = take_picture()
        print("Image captured from camera at Pose 1.")
        show_image(image)
        save_image(image, 'captured_image1.jpg')

        height = run(image)
        print(f"Detected height: {height}")

        if height < (middle_video-100):
            Z_HEIGHT -= 0.01

            target_pose1 = [target_x1, target_y1, Z_HEIGHT] + FIXED_ORIENTATION
            rtde_c.moveL(target_pose1, SPEED, ACCELERATION)
            stop_move(rtde_c)

        elif height > (middle_video+100):
            Z_HEIGHT += 0.01
            center_height = True
        
        else:
            print(f"Height is within acceptable range: {Z_HEIGHT}")
            center_height = True






    rtde_c.moveL(safe_pose1, SPEED, ACCELERATION) 


    time.sleep(1)


except Exception as e:
    print(f"An error occurred: {e}")

finally:
    # --- Cleanup ---
    # open(servol) 
    time.sleep(1)
    print("Cleaning up resources...")
    stop_image()
    # stop(servol)


    rtde_c.disconnect()
    print("Program finished.")

