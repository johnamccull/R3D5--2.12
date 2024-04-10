import rtde_control, rtde_receive
import time
import keyboard 

# PARAMETERS
# Robot IP address
IP_UR5 = "169.254.157.0"
USE_ROBOT = True

# Keyboard control directions and commands
KEY_XM = 's'
KEY_XP = 'f'
KEY_YM = 'd'
KEY_YP = 'e'

KEY_ACW = 'j'
KEY_CW = 'l'
KEY_ZP = 'i'
KEY_ZM = 'k'

KEY_QUIT = 'q'

# Set position increments
INC_PLANE = 0.01
INC_HEIGHT = 0.01
INC_ROT = 0.01

SPEED_L = 3.0 #0.5 #0.25
ACCEL_L = 0.1 #25

# VARIABLES
# Current position of the robot
current_poseL_d = [0, 0, 0, 0, 0, 0]


def setup():
    # Connect to the robot
    if USE_ROBOT:
        # Connect to the robot
        rtde_c =  rtde_control.RTDEControlInterface(IP_UR5)
        rtde_r = rtde_receive.RTDEReceiveInterface(IP_UR5)

        # Get the current position of the robot
        current_poseL_d[:] = rtde_r.getActualTCPPose()
    else:
        rtde_c = None
        rtde_r = None

        current_poseL_d[:] = [10, 10, 10, 10, 10, 10]

    print('Setup complete. Robot connected.')

    return rtde_c, rtde_r


def loop(rtde_c, rtde_r):
    ## POLLING LOOP
    print('About to enter manual control loop. Press q to quit.')
    
    while True:
        if keyboard.is_pressed(KEY_XM):
            current_poseL_d[0] -= INC_PLANE

        elif keyboard.is_pressed(KEY_XP):
            current_poseL_d[0] += INC_PLANE

        elif keyboard.is_pressed(KEY_YP):
            current_poseL_d[1] += INC_PLANE

        elif keyboard.is_pressed(KEY_YM):
            current_poseL_d[1] -= INC_PLANE

        elif keyboard.is_pressed(KEY_ZP):
            current_poseL_d[2] += INC_HEIGHT

        elif keyboard.is_pressed(KEY_ZM):
            current_poseL_d[2] -= INC_HEIGHT

        elif keyboard.is_pressed(KEY_ACW):
            current_poseL_d[5] += INC_ROT

        elif keyboard.is_pressed(KEY_CW):
            current_poseL_d[5] -= INC_ROT
        
        elif keyboard.is_pressed(KEY_QUIT): 
            print('Quit key pressed')
            break  # finishing the loop

        # Move to desired setpoint
        # Move asynchronously in cartesian space to target, we specify asynchronous behavior by setting the async parameter to
        # 'True'. Try to set the async parameter to 'False' to observe a default synchronous movement, which cannot be stopped
        # by the stopL function due to the blocking behaviour.
        rtde_c.moveL(current_poseL_d, SPEED_L, ACCEL_L, False)
        print(current_poseL_d)

        time.sleep(0.01) # Run at 100 Hz
        

if __name__ == "__main__":
    # SETUP
    rtde_c, rtde_r = setup()

    # LOOP
    loop(rtde_c, rtde_r)



# Perhaps: move asynchronously parallel to windows of rubble until high IR detected and stop there? or store highest IR value and move back to that

# USEFUL FUNCTIONS: 
# rtde_c.moveUntilContact(speed)
# moveJ - in joint space, moveL - in cartesian space
# moveJ_IK - specify a pose that a robot will move to linearly in joint space
# ***moveL - can set pos + speed + acceleration. Moves linearly in tool space
#       - can specify "path" - 2d vector of doubles that also includes speeds and accelerations
# ***movePath - move to each waypoint specified in a given path   