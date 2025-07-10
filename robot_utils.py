
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


def search(mid_n, rtde_c, start_x, start_y, Z_HEIGHT, FIXED_ORIENTATION, SPEED, ACCELERATION):
    """
    
    """
    last_pos = 0
    min_dist = 10000


    list_of_positions = []
    max_y = start_y

    tracked_needle_id = None 
    last_known_needle_pos = None

    while max_y > -0.15:
        move = [start_x, max_y, Z_HEIGHT] + FIXED_ORIENTATION
        rtde_c.moveL(move, SPEED, ACCELERATION)
        stop_move(rtde_c)

        image = take_picture()
        clustering, sorted_index, lines = run(image)
        point = points(lines)
        print("1")

        if lines == -1:
            max_y -= 0.02
            tracked_needle_id = None 
            last_known_needle_pos = None
            continue
        
        print("2")
        avg_list = [sum(point[idx][0] for idx in group) / len(group) for group in sorted_index]
        print(f"\n Average positions: {avg_list} \n")

        current_frame_needles = []
        for i in avg_list:
            if mid_n - 100 < i < mid_n + 100:
                current_frame_needles.append(i)
                last_pos = i
        
        if current_frame_needles:
            if tracked_needle_id is None:
                tracked_needle_id = hash(current_frame_needles[0])
                list_of_positions.append(max_y)
                last_known_needle_pos = current_frame_needles[0]

            else:
                list_of_positions.append(max_y)
                last_known_needle_pos = current_frame_needles[0]

        else:
            if tracked_needle_id is not None:
                print(f"Needle lost at height {max_y}, last known position: {last_known_needle_pos}")
            
            tracked_needle_id = None
            last_known_needle_pos = None

                
        max_y -= 0.02

        print(f"points image: {point}")
        print(f"sorted_index: {sorted_index}")
        print(f"clustering: {clustering}")

    
    return list_of_positions
