
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


def find_height(rtde_c, target_x1, target_y1, Z_HEIGHT, FIXED_ORIENTATION, SPEED, ACCELERATION):
    """    Find the height of the object by adjusting the robot's position based on camera feedback.
    """
    mid = 540

    center_height = False

    while not center_height:
        image = take_picture()
        print("Image captured from camera at Pose 1.")
        show_image(image)
        save_image(image, 'captured_image1.jpg')

        height = run(image)
        if height == -1:
            print("No lines detected after angle filtering. Skipping clustering.")
            return Z_HEIGHT
            

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

    return Z_HEIGHT

