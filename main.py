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

    target_x = -0.5
    target_y = 0.1

    target_x2 = -0.5
    target_y2 = -0.15

    target_pose0 = [target_x0, target_y0, target_z0] + FIXED_ORIENTATION

    target_pose1 = [target_x, target_y, Z_HEIGHT] + FIXED_ORIENTATION
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



    Z_HEIGHT, bo = find_height(safe_mid, rtde_c, target_x, target_y, Z_HEIGHT, FIXED_ORIENTATION, SPEED, ACCELERATION)
    print(f"Final Z Height after adjustment: {Z_HEIGHT}")

    correct_pos = []
    last_pos = 0

    while target_y > -0.15:

        while not bo:
            target_y -= 0.02

            target_pose1 = [target_x, target_y, Z_HEIGHT] + FIXED_ORIENTATION
            rtde_c.moveL(target_pose1, SPEED, ACCELERATION)
            stop_move(rtde_c)

            Z_HEIGHT, bo = find_height(safe_mid, rtde_c, target_x, target_y, Z_HEIGHT, FIXED_ORIENTATION, SPEED, ACCELERATION)

        center = False
        while not center:

            image = take_picture()
            clustering, sorted_index, lines = run(image)
            point = points(lines)
            print(f"points image: {point}")

            if clustering > 1:
                print(f"\n last_pos: {last_pos} \n")

                # Re-initialize for each detection frame
                target_needle_pos_from_right = -1
                min_positive_dist = float('inf')

                avg_list = [sum(group[idx][0] for idx in range(len(group))) / len(group) for group in sorted_index] # Corrected way to sum group points
                print(f"Average positions: {avg_list}")

                # Find the needle that is to the right (greater x-coordinate) of the last_pos
                # and is the "closest" among those on the right.
                for i in avg_list:
                    dist = i - last_pos
                    if dist > 0 and dist < min_positive_dist: # Only consider positive distances (right side)
                        min_positive_dist = dist
                        target_needle_pos_from_right = i

                if target_needle_pos_from_right != -1:
                    needle_pos = target_needle_pos_from_right
                    print(f"Targeting new needle from the right: {needle_pos}")

                    if needle_pos > (mid_n + 50) or needle_pos < (mid_n - 50): # Increased threshold for adjustment
                        print(f"Needle is not centered, adjusting position: {needle_pos}")
                        # quit_key() # Removed quit_key for automatic operation
                        target_y, last_pos = adjust_pos(needle_pos, mid_n, target_x, target_y, Z_HEIGHT, FIXED_ORIENTATION, rtde_c, SPEED, ACCELERATION)
                    else:
                        print(f"Needle is centered at: {needle_pos}")
                        correct_pos.append(needle_pos)
                        last_pos = needle_pos # IMPORTANT: Update last_pos to the *centered* needle
                        center = True # Mark as centered for *this* needle, exit inner loop
                else:
                    # This means clustering > 1, but no new needle found to the right of last_pos.
                    # It might mean we are past the last needle, or the detection is noisy.
                    print("Clustering > 1, but no new needle found to the right of last_pos. Moving slightly to search.")
                    target_y -= 0.005 # Small step to try and bring a new needle into view
                    target_pose1 = [target_x, target_y, Z_HEIGHT] + FIXED_ORIENTATION
                    rtde_c.moveL(target_pose1, SPEED, ACCELERATION)
                    stop_move(rtde_c)
                    # Do not set center=True or break, allow loop to re-evaluate detections.

            elif clustering == 1: # Explicitly checking for a single cluster now
                print("\n Single cluster detected \n")
                needle_pos = run_center(image)
                print(f"Needle position: {needle_pos}")

                if needle_pos == -1:
                    print("No valid center detected for single cluster. Moving slightly.")
                    target_y += 0.005 # Nudge in positive y (towards the right/new needles)
                    target_pose1 = [target_x, target_y, Z_HEIGHT] + FIXED_ORIENTATION
                    rtde_c.moveL(target_pose1, SPEED, ACCELERATION)
                    stop_move(rtde_c)
                    continue # Continue to next iteration of while not center to re-take picture

                if needle_pos > (mid_n + 25) or needle_pos < (mid_n - 25):
                    target_y, last_pos = adjust_pos(needle_pos, mid_n, target_x, target_y, Z_HEIGHT, FIXED_ORIENTATION, rtde_c, SPEED, ACCELERATION)
                else:
                    print(f"Needle is centered at: {needle_pos}")
                    correct_pos.append(needle_pos)
                    last_pos = needle_pos # IMPORTANT: Update last_pos after centering
                    center = True # Mark as centered for *this* needle

            elif clustering == -1: # No clusters detected at all
                print("No clusters detected (-1). Moving slightly to search.")
                target_y -= 0.005 # Move slightly along scan direction to find something
                target_pose1 = [target_x, target_y, Z_HEIGHT] + FIXED_ORIENTATION
                rtde_c.moveL(target_pose1, SPEED, ACCELERATION)
                stop_move(rtde_c)
                # Do not set center=True, allow loop to re-evaluate

        # AFTER a needle has been centered (center == True for current needle),
        # move to the next scan position to find the next needle.
        print(f"Needle handled. Moving to next scan position. Current target_y: {target_y}")
        target_y -= 0.02 # Move further along the scan path to find the next needle
        bo = False # Reset bo to re-check Z_HEIGHT for the new target_y position.
        # The while not bo loop will then execute to adjust Z if needed for the new target_y.








    rtde_c.moveL(safe_pose1, SPEED, ACCELERATION) 


    time.sleep(1)


except Exception as e:
    print(f"An error occurred: {e}")

finally:
    correct_pos
    # --- Cleanup ---
    # open(servol) 
    time.sleep(1)
    print("Cleaning up resources...")
    stop_image()
    # stop(servol)


    rtde_c.disconnect()
    print("Program finished.")

