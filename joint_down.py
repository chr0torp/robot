import time
import math
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

from robot_utils import *
from move import *


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

    close(servol)
    open(servol)

    # --- Get Initial Position (Optional but good practice) ---
    initial_pose = rtde_r.getActualTCPPose()
    print(f"Initial TCP Pose: {initial_pose}")
    wait_for_key("Press Enter to continue after checking initial pose...")

    # --- Define Target X, Y Coordinates ---
    target_x1 = -0.5
    target_y1 = -0.15

    target_x2 = -0.5
    target_y2 = -0.1

    target_x3 = -0.5
    target_y3 = -0.05

    target_x4 = -0.5
    target_y4 = 0.0

    target_x5 = -0.5
    target_y5 = 0.05

    target_x6 = -0.5
    target_y6 = 0.1

    target_x7 = -0.5
    target_y7 = 0.15


    # --- Construct Target Poses ---
    target_pose1 = [target_x1, target_y1, Z_HEIGHT] + FIXED_ORIENTATION
    safe_pose1 = safe_pos(target_pose1)

    target_pose2 = [target_x2, target_y2, Z_HEIGHT] + FIXED_ORIENTATION
    safe_pose2 = safe_pos(target_pose2)

    target_pose3 = [target_x3, target_y3, Z_HEIGHT] + FIXED_ORIENTATION
    safe_pose3 = safe_pos(target_pose3)

    target_pose4 = [target_x4, target_y4, Z_HEIGHT] + FIXED_ORIENTATION
    safe_pose4 = safe_pos(target_pose4)

    target_pose5 = [target_x5, target_y5, Z_HEIGHT] + FIXED_ORIENTATION
    safe_pose5 = safe_pos(target_pose5)

    target_pose6 = [target_x6, target_y6, Z_HEIGHT] + FIXED_ORIENTATION
    safe_pose6 = safe_pos(target_pose6)
    

    # --- Execute Moves ---

    print(f"Moving to Pose 1: {target_pose1}")
    rtde_c.moveL(safe_pose1, SPEED, ACCELERATION) 
    rtde_c.moveL(target_pose1, SPEED, ACCELERATION)
    stop_move(rtde_c)
    print("Reached Pose 1.")
    close(servol) 
    rtde_c.moveL(safe_pose1, SPEED, ACCELERATION) 
    open(servol) 
    # quit_key()

    print(f"Moving to Pose 2: {target_pose2}")
    rtde_c.moveL(safe_pose2, SPEED, ACCELERATION) 
    rtde_c.moveL(target_pose2, SPEED, ACCELERATION)
    stop_move(rtde_c)
    print("Reached Pose 2.")
    close(servol) 
    rtde_c.moveL(safe_pose2, SPEED, ACCELERATION)  
    open(servol)  

    print(f"Moving to Pose 3: {target_pose3}")
    rtde_c.moveL(safe_pose3, SPEED, ACCELERATION) 
    rtde_c.moveL(target_pose3, SPEED, ACCELERATION)
    stop_move(rtde_c)
    print("Reached Pose 3.")
    close(servol)  
    rtde_c.moveL(safe_pose3, SPEED, ACCELERATION) 
    open(servol)  

    print(f"Moving to Pose 4: {target_pose4}")
    rtde_c.moveL(safe_pose4, SPEED, ACCELERATION) 
    rtde_c.moveL(target_pose4, SPEED, ACCELERATION)
    stop_move(rtde_c)
    print("Reached Pose 4.")
    close(servol) 
    rtde_c.moveL(safe_pose4, SPEED, ACCELERATION) 
    open(servol) 

    print(f"Moving to Pose 5: {target_pose5}")
    rtde_c.moveL(safe_pose5, SPEED, ACCELERATION) 
    rtde_c.moveL(target_pose5, SPEED, ACCELERATION)
    stop_move(rtde_c)
    print("Reached Pose 5.")
    close(servol)  
    rtde_c.moveL(safe_pose5, SPEED, ACCELERATION)  
    open(servol) 

    print(f"Moving to Pose 6: {target_pose6}")
    rtde_c.moveL(safe_pose6, SPEED, ACCELERATION)
    rtde_c.moveL(target_pose6, SPEED, ACCELERATION)
    stop_move(rtde_c)
    print("Reached Pose 6.")
    close(servol)
    rtde_c.moveL(safe_pose6, SPEED, ACCELERATION)
    open(servol)



except Exception as e:
    print(f"An error occurred: {e}")

finally:
    # --- Disconnect ---
    stop(servol)
    if 'rtde_c' in locals() and rtde_c.isConnected():
        print("Stopping script and disconnecting.")
        rtde_c.stopScript()
    print("Program finished.")

