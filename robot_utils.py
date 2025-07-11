
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

from robot_utils import *
from servo_utils import *
from camara_utils import *
from detect_utils import *



def stop_move(rtde_c):
    """
    Stop the robot's motion.
    """
    rtde_c.stopL()
    rtde_c.stopJ() 


def wait_for_key(prompt="Press Enter to continue..."):
    input(prompt)

def quit_key(prompt="Press q then enter to quit, press enter to continue..."):
    """
    Wait for the user to press Enter to quit or space to continue.
    This version is platform-independent.
    """
    key = input(prompt)
    if key.strip() == "q":
        exit(0)
    elif key.strip() == "":
        print("Continuing...")
    else:
        print("Unrecognized input, continuing...")



def safe_pos(dest):
    """
    """
    return [dest[0], dest[1], dest[2] + 0.1, dest[3], dest[4], dest[5]]


def find_height(mid, rtde_c, target_x1, target_y1, Z_HEIGHT, FIXED_ORIENTATION, SPEED, ACCELERATION):
    """    Find the height of the object by adjusting the robot's position based on camera feedback.
    """

    center_height = False

    while not center_height:
        image = take_picture()

        height = run_height(image)
        if height == -1:
            print("No lines detected after angle filtering. Skipping clustering.")
            return Z_HEIGHT, False
            

        print(f"Detected height: {height}")

        if height > (mid+25):
            if height < (mid+100):
                Z_HEIGHT -= 0.005
            else:
                Z_HEIGHT -= 0.01

            target_pose1 = [target_x1, target_y1, Z_HEIGHT] + FIXED_ORIENTATION
            rtde_c.moveL(target_pose1, SPEED, ACCELERATION)
            stop_move(rtde_c)

        elif height < (mid-25):
            if height > (mid-100):
                Z_HEIGHT += 0.005
            else:
                Z_HEIGHT += 0.01

            target_pose1 = [target_x1, target_y1, Z_HEIGHT] + FIXED_ORIENTATION
            rtde_c.moveL(target_pose1, SPEED, ACCELERATION)
            stop_move(rtde_c)
        
        else:
            print(f"Height is within acceptable range: {Z_HEIGHT}")
            center_height = True

    return Z_HEIGHT, True

def adjust_pos(needle_pos, mid_n, target_x1, target_y1, Z_HEIGHT, FIXED_ORIENTATION, rtde_c, SPEED, ACCELERATION):
    """
    Adjusts the needle position based on its current position relative to the midpoint.
    """
    if needle_pos > (mid_n + 25):
        if needle_pos < (mid_n + 150):
            target_y1 -= 0.0025
        else:
            target_y1 -= 0.005

        target_pose1 = [target_x1, target_y1, Z_HEIGHT] + FIXED_ORIENTATION
        rtde_c.moveL(target_pose1, SPEED, ACCELERATION)
        stop_move(rtde_c)

    elif needle_pos < (mid_n - 25):
        if needle_pos > (mid_n - 150):
            target_y1 += 0.0025
        else:
            target_y1 += 0.005

        target_pose1 = [target_x1, target_y1, Z_HEIGHT] + FIXED_ORIENTATION
        rtde_c.moveL(target_pose1, SPEED, ACCELERATION)
        stop_move(rtde_c)
    
    return target_y1

def merge_positions(positions):

    treshold = 0.011
    if not positions:
        return []
    
    merged_positions = []
    sorted_list = sorted(positions)
    current_group = [sorted_list[0]]

    for pos in sorted_list[1:]:
        if abs(pos - current_group[-1]) <= treshold:
            current_group.append(pos)
        else:
            merged_positions.append(sum(current_group) / len(current_group))
            current_group = [pos]

    if current_group:
        merged_positions.append(sum(current_group) / len(current_group))
    
    return merged_positions



def search(mid_n, rtde_c, start_x, start_y, Z_HEIGHT, FIXED_ORIENTATION, SPEED, ACCELERATION):
    """
    
    """
    last_pos = 0

    list_of_positions = []
    max_y = start_y


    while max_y > -0.15:
        move = [start_x, max_y, Z_HEIGHT] + FIXED_ORIENTATION
        rtde_c.moveL(move, SPEED, ACCELERATION)
        stop_move(rtde_c)

        image = take_picture()
        clustering, sorted_index, lines = run(image)
        point = points(lines)
        print("1")
        print(f"lines: {lines}")

        if len(point) == 0:
            print("No lines detected after angle filtering. Skipping clustering.")
            max_y -= 0.005
            last_pos = 0
            continue
        
        print("2")
        avg_list = [sum(point[idx][0] for idx in group) / len(group) for group in sorted_index]
        print(f"\n Average positions: {avg_list} \n")

        best_candidate_x = -1 
        best_candidate_dist = 10000

        for i in avg_list:
            if mid_n < i < mid_n + 100:
                if last_pos != 0:
                    dist = abs(i - last_pos)
                    if dist < best_candidate_dist:
                        best_candidate_dist = dist
                        best_candidate_x = i
                else:
                    best_candidate_x = i
                    break

        if best_candidate_x != -1: 
            list_of_positions.append(max_y) 
            last_pos = best_candidate_x 
        else:
            last_pos = 0

                
        max_y -= 0.005


    move = [start_x, max_y, Z_HEIGHT] + FIXED_ORIENTATION
    safe = safe_pos(move)
    rtde_c.moveL(safe, SPEED, ACCELERATION)

    my_formatted_list = ['%.2f' % elem for elem in list_of_positions]
    print(f"Formatted positions: {my_formatted_list}")
    no_duplicates = list(set(my_formatted_list))
    print(f"Formatted positions without duplicates: {no_duplicates}")
    no_duplicates_float = [float(s) for s in no_duplicates]
    print(f"Formatted positions without duplicates as float: {no_duplicates_float}")
    merge = merge_positions(no_duplicates_float)
    print(f"Merged positions: {merge}")

    return merge


if __name__ == "__main__":
    lst = [0.07, 0.05, 0.04, 0.01, 0.02]
    lst2 = [0.07, 0.05, 0.02, 0.01, 0.04]
    lst3 = [-0.06, -0.01, -0.12, 0.02]

    merged = merge_positions(lst)
    merged2 = merge_positions(lst2)
    merged3 = merge_positions(lst3)

    print(f"merged {merged}")
    print(f"merged2 {merged2}")
    print(f"merged3 {merged3}")