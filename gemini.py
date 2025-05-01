import rtde_control
import rtde_receive
import time
import math

# --- CONFIGURATION ---
ROBOT_IP = "192.168.1.101"  # !!! IMPORTANT: Replace with your robot's actual IP address !!!
JOINT_SPEED = 0.5  # rad/s
JOINT_ACCELERATION = 1.0  # rad/s^2
JOINT_INDEX_TO_MOVE = 0  # Which joint to move (0=base, 1=shoulder, ..., 5=wrist 3)
MOVE_AMOUNT_RADIANS = 0.1 # How much to move the joint (in radians, approx 5.7 degrees)
# --- END CONFIGURATION ---

rtde_c = None # Initialize control interface variable outside try block

try:
    # --- Connect ---
    print(f"Connecting to robot at {ROBOT_IP}...")
    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
    rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
    print("Connected successfully.")

    # --- Get Current Position ---
    print("Reading current joint positions...")
    current_q = rtde_r.getActualQ()
    if not current_q or len(current_q) != 6:
         raise ValueError("Could not read valid joint positions from the robot.")
    print(f"Current joints (radians): {[f'{q:.3f}' for q in current_q]}")

    # --- Define Target Position ---
    # Create a copy of the current positions to define the target
    target_q = current_q[:]
    # Modify the desired joint angle
    target_q[JOINT_INDEX_TO_MOVE] += MOVE_AMOUNT_RADIANS
    print(f"Target joints (radians):  {[f'{q:.3f}' for q in target_q]}")
    print(f"Moving joint {JOINT_INDEX_TO_MOVE} by {math.degrees(MOVE_AMOUNT_RADIANS):.2f} degrees...")

    # --- Execute the Move ---
    # moveJ(q, speed, acceleration, async)
    # Using async=False (default) makes the script wait here until the move is complete.
    rtde_c.moveJ(target_q, JOINT_SPEED, JOINT_ACCELERATION, False)
    print("Movement finished.")

    # Optional: Add a small pause
    time.sleep(1)

    # Optional: Move back to the starting position
    # print("Moving back to start position...")
    # rtde_c.moveJ(current_q, JOINT_SPEED, JOINT_ACCELERATION)
    # print("Returned to start position.")

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    # --- Clean Up ---
    if rtde_c and rtde_c.isConnected():
        print("Stopping RTDE control script...")
        rtde_c.stopScript()
        print("Script stopped.")
    else:
        print("Control interface not connected or already stopped.")