import time
import math
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

from robot_utils import *
from servo_utils import *
from camara_utils import *
from detect_utils import *

mid = 540
safe_mid = 800

mid_n = 1040

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
    # wait_for_key()

    # image = take_picture()
    # print("Image captured from camera.")
    # show_image(image)
    # save_image(image, 'captured_image1.jpg')

    # --- Move to Initial Position ---
    target_x0 = -0.5
    target_y0 = 0.0
    target_z0 = 0.60

    # target_x1 = -0.5
    # target_y1 = -0.15

    target_x1 = -0.5
    target_y1 = 0.1

    target_x2 = -0.5
    target_y2 = -0.15

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



    Z_HEIGHT, bo = find_height(safe_mid, rtde_c, target_x1, target_y1, Z_HEIGHT, FIXED_ORIENTATION, SPEED, ACCELERATION)
    print(f"Final Z Height after adjustment: {Z_HEIGHT}")

    if not bo:
        print("No valid height detected, exiting.")
        rtde_c.disconnect()
        exit(1)

    if bo:

        center = False
        last_pos = 0
        while not center:
            
            image = take_picture()
            clustering, sorted_index, lines = run(image)
            point = points(lines)
            print(f"points image: {point}")


            if clustering > 1:
                avg_list = [sum(point[idx] for idx in group) / len(group) for group in sorted_index]
                print(f"Average positions: {avg_list}")
                print(f"last_pos: {last_pos}")
                print(f"Clustering detected: {clustering}")
                break

            if clustering < 2 and clustering != -1:
                print("\n go go go \n")
                needle_pos = run_center(image)
                print(f"Needle position: {needle_pos}")

                if needle_pos == -1:
                    print("No valid center detected")
                    target_y1 += 0.005

                    target_pose1 = [target_x1, target_y1, Z_HEIGHT] + FIXED_ORIENTATION
                    rtde_c.moveL(target_pose1, SPEED, ACCELERATION)
                    stop_move(rtde_c)
                    
                    image = take_picture()
                    clustering =  run(image)

                    if clustering == -1 or clustering > 1:
                        break
                    else:
                        continue

                elif needle_pos > (mid_n+25):
                    last_pos = needle_pos
                    if needle_pos < (mid_n+150):
                        target_y1 -= 0.0025
                    else:
                        target_y1 -= 0.01

                    target_pose1 = [target_x1, target_y1, Z_HEIGHT] + FIXED_ORIENTATION
                    rtde_c.moveL(target_pose1, SPEED, ACCELERATION)
                    stop_move(rtde_c)

                elif needle_pos < (mid_n-25):
                    last_pos = needle_pos
                    if needle_pos > (mid_n-150):
                        target_y1 += 0.0025
                    else:
                        target_y1 += 0.01
                    
                    target_pose1 = [target_x1, target_y1, Z_HEIGHT] + FIXED_ORIENTATION
                    rtde_c.moveL(target_pose1, SPEED, ACCELERATION)
                    stop_move(rtde_c)
                        
                else:
                    print(f"Needle is centered at: {needle_pos}")
                    center = True
                    






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

