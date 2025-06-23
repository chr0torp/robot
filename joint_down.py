import time
import math
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

from robot_utils import *


# --- Configuration ---
ROBOT_IP = "192.168.1.102"  # Replace with your robot's actual IP address
Z_HEIGHT = 0.3             # Desired constant Z height (in meters)
SPEED = 0.1                # TCP speed (m/s)
ACCELERATION = 0.01         # TCP acceleration (m/s^2)

# Define the fixed orientation (tool's Z-axis points down along base -Z)
# Rotation of pi radians (180 degrees) around the base X-axis
FIXED_ORIENTATION = [math.pi, 0, 0]
# --- End Configuration ---

try:
    # --- Connect to Robot ---
    rtde_c = RTDEControl(ROBOT_IP)
    rtde_r = RTDEReceive(ROBOT_IP)
    print("Successfully connected to robot.")

    # --- Get Initial Position (Optional but good practice) ---
    initial_pose = rtde_r.getActualTCPPose()
    print(f"Initial TCP Pose: {initial_pose}")
    wait_for_key("Press Enter to continue after checking initial pose...")

    # --- Define Target X, Y Coordinates ---
    # Example: Move to X=0.4, Y=-0.2 while keeping Z=Z_HEIGHT and orientation fixed
    target_x1 = -0.5
    target_y1 = -0.2

    # Example: Move to X=0.5, Y=0.1 while keeping Z=Z_HEIGHT and orientation fixed
    target_x2 = -0.5
    target_y2 = 0.0

    target_x3 = -0.5
    target_y3 = 0.2

    # --- Construct Target Poses ---
    target_pose1 = [target_x1, target_y1, Z_HEIGHT] + FIXED_ORIENTATION
    target_pose2 = [target_x2, target_y2, Z_HEIGHT] + FIXED_ORIENTATION
    target_pose3 = [target_x3, target_y3, Z_HEIGHT] + FIXED_ORIENTATION


    # --- Execute Moves ---

    print(f"Moving to Pose 1: {target_pose1}")
    rtde_c.moveL(target_pose1, SPEED, ACCELERATION)
    rtde_c.stopL()
    rtde_c.stopJ()  # Ensure the robot stops any ongoing joint movement
    print("Reached Pose 1.")
    wait_for_key("Press Enter to continue after reaching Pose 1...")
    

    print(f"Moving to Pose 2: {target_pose2}")
    rtde_c.moveL(target_pose2, SPEED, ACCELERATION)
    rtde_c.stopL()
    rtde_c.stopJ() 
    print("Reached Pose 2.")
    wait_for_key("Press Enter to continue after reaching Pose 2...")

    print(f"Moving to Pose 3: {target_pose3}")
    rtde_c.moveL(target_pose3, SPEED, ACCELERATION)
    rtde_c.stopL()
    rtde_c.stopJ()
    print("Reached Pose 3.")
    wait_for_key("Press Enter to continue after reaching Pose 3...")


except Exception as e:
    print(f"An error occurred: {e}")

finally:
    # --- Disconnect ---
    if 'rtde_c' in locals() and rtde_c.isConnected():
        print("Stopping script and disconnecting.")
        rtde_c.stopScript()
    print("Program finished.")

