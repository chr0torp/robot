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
Z_HEIGHT = 0.36             # Desired constant Z height (in meters)
SAFE_Z_HEIGHT = 0.36

SPEED = 0.3  
ACCELERATION = 0.01         # TCP acceleration (m/s^2)

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
    safe_pose0 = safe_pos(target_pose0)


    target_pose1 = [target_x, target_y, Z_HEIGHT] + FIXED_ORIENTATION
    safe_pose1 = safe_pos(target_pose1)


    # print(f"Moving to initial position: {target_pose0}")
    # rtde_c.moveL(target_pose0, SPEED, ACCELERATION)
    # stop_move(rtde_c)
    # print("Reached initial position.")


    # print(f"Moving to Pose 1: {target_pose1}")
    # rtde_c.moveL(safe_pose1, SPEED, ACCELERATION) 
    # rtde_c.moveL(target_pose1, SPEED, ACCELERATION)
    # stop_move(rtde_c)
    # print("Reached Pose 1.")



    # Z_HEIGHT, bool = find_height(safe_mid, rtde_c, target_x, target_y, Z_HEIGHT, FIXED_ORIENTATION, SPEED, ACCELERATION)
    # print(f"Final Z Height after adjustment: {Z_HEIGHT}")


    # while not bool:
    #     target_y -= 0.02

    #     target_pose1 = [target_x, target_y, Z_HEIGHT] + FIXED_ORIENTATION
    #     rtde_c.moveL(target_pose1, SPEED, ACCELERATION)
    #     stop_move(rtde_c)

    #     Z_HEIGHT, bool = find_height(safe_mid, rtde_c, target_x, target_y, Z_HEIGHT, FIXED_ORIENTATION, SPEED, ACCELERATION)


    # if bool:
    #     correct_pos_x = search(mid_n, rtde_c, target_x, target_y, Z_HEIGHT, FIXED_ORIENTATION, SPEED, ACCELERATION)

    # correct_pos_x = [-0.14, -0.12, -0.09, -0.06, -0.01, 0.02, 0.06]
    correct_pos_x = [0.06]
    # correct_pos_x = [0.02, 0.06]

    for i in correct_pos_x:
        print(f"Processing position: {i}")


        target_pose1 = [target_x, i, SAFE_Z_HEIGHT] + FIXED_ORIENTATION
        safe_pose1 = safe_pos(target_pose1)
        rtde_c.moveL(safe_pose1, SPEED, ACCELERATION)
        rtde_c.moveL(target_pose1, SPEED, ACCELERATION)
        stop_move(rtde_c)

        Z_HEIGHT, bool = find_height(safe_mid, rtde_c, target_x, i, SAFE_Z_HEIGHT, FIXED_ORIENTATION, SPEED, ACCELERATION)
        target_pose2 = [target_x2, i, Z_HEIGHT] + FIXED_ORIENTATION
        rtde_c.moveL(target_pose2, SPEED, ACCELERATION)
        stop_move(rtde_c)

        center = False
        center_point = mid_n
        min_distance = 10000
        best_candidate_x = 0.0
        y = i

        while not center:
            
            image = take_picture()
            clustering, sorted_index, lines = run(image)
            point = points(lines)
            print(f"points image: {point}")


            if clustering > 1:
                avg_list = [sum(point[idx][0] for idx in group) / len(group) for group in sorted_index]
                print(f"Average positions: {avg_list}")

                for i in avg_list:
                    if abs(i - center_point) < min_distance:
                        min_distance = abs(i - center_point)
                        best_candidate_x = i

                if not (center_point - 25 < best_candidate_x < center_point + 25):
                    y = adjust_pos(best_candidate_x, center_point, target_x, y, Z_HEIGHT, FIXED_ORIENTATION, rtde_c, SPEED, ACCELERATION)
                    min_distance = 10000

                elif center_point - 25 < best_candidate_x < center_point + 25:
                    print("Best candidate within range, stopping search.")
                    center = True
                    continue

            if clustering == 1:
                print("Only one cluster detected, using its position.")
                best_candidate_x = point[0][0]

                if not (center_point - 25 < best_candidate_x < center_point + 25):
                    y = adjust_pos(best_candidate_x, center_point, target_x, y, Z_HEIGHT, FIXED_ORIENTATION, rtde_c, SPEED, ACCELERATION)
                    min_distance = 10000

                elif center_point - 25 < best_candidate_x < center_point + 25:
                    print("Best candidate within range, stopping search.")
                    center = True
                    continue            

            if clustering == 0:
                print("No clusters detected, stopping search.")
                center = True
                continue


        Z_HEIGHT, bool = find_height(mid, rtde_c, target_x, y, Z_HEIGHT, FIXED_ORIENTATION, SPEED, ACCELERATION)
        
        true_depth = False
        while not true_depth:
            
            degree_list = [10, 15, 10]

            center_point = mid_n
            min_distance = 10000
            best_candidate_x = 0.0

            for degree in degree_list:
                print(f"Time to move ---------- {degree} degrees")

                move_Wrist_3(degree, rtde_c, rtde_r, SPEED, ACCELERATION)
                image = take_picture()
                clustering, sorted_index, lines = run(image)
                point = points(lines)

                avg_list = [sum(point[idx][0] for idx in group) / len(group) for group in sorted_index]
                print(f"Average positions: {avg_list}")

                for i in avg_list:
                    if abs(i - center_point) < min_distance:
                        min_distance = abs(i - center_point)
                        best_candidate_x = i

                min_distance = 10000
                center_point = best_candidate_x

            if center_point < mid_n - 25:
                if center_point < mid_n - 100:
                    target_x += 0.01
                else:
                    target_x += 0.0025

            elif center_point > mid_n + 25:
                if center_point > mid_n + 100:
                    target_x -= 0.01
                else:
                    target_x -= 0.0025

            else:
                print("Best candidate within range, stopping search.")
                true_depth = True
                quit_key()
                continue
            
            print(f"Time to move ----------")
            # quit_key()
            # sum_of_degrees = sum(degree_list)
            # move_Wrist_3(-sum_of_degrees, rtde_c, rtde_r, SPEED, ACCELERATION)

            target_pose = [target_x, y, Z_HEIGHT] + FIXED_ORIENTATION
            rtde_c.moveL(target_pose, SPEED, ACCELERATION)
            stop_move(rtde_c)




        print(f"points image: {point}")
        show_image(image)
        quit_key()

        move_Wrist_3(-10, rtde_c, rtde_r, SPEED, ACCELERATION)


    rtde_c.moveL(safe_pose0, SPEED, ACCELERATION) 


    time.sleep(1)


except Exception as e:
    print(f"An error occurred: {e}")

finally:
    print(f"\n correct_pos_x: {correct_pos_x} \n")
    # --- Cleanup ---
    # open(servol) 
    time.sleep(1)
    print("Cleaning up resources...")
    stop_image()
    # stop(servol)


    rtde_c.disconnect()
    print("Program finished.")

