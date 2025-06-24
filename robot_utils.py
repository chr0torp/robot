
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive



def stop_move(rtde_c):
    """
    Stop the robot's motion.
    """
    rtde_c.stopL()
    rtde_c.stopJ() 


def wait_for_key(prompt="Press Enter to continue..."):
    input(prompt)


def safe_pos(dest):
    """
    """
    return [dest[0], dest[1], dest[2] + 0.1, dest[3], dest[4], dest[5]]


