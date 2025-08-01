import time
import math
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

ROBOT_IP = "192.168.1.102"

try:
    rtde_c = RTDEControl(ROBOT_IP)
    rtde_r = RTDEReceive(ROBOT_IP)
    print("Successfully connected to robot.")

    initial_pose = rtde_r.getActualTCPPose()
    print(f"Initial TCP Pose: {initial_pose}")

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    # --- Disconnect ---
    if 'rtde_c' in locals() and rtde_c.isConnected():
        print("Stopping script and disconnecting.")
        rtde_c.stopScript()
    print("Program finished.")