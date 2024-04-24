import time
import PS4_Control as ps4
import math

# PARAMETERS
# Robot IP address
IP_UR5 = "169.254.157.0"
USE_ROBOT = False #True
USE_CONTROLLER = True
USE_GRIPPER = False

if USE_ROBOT:
    import rtde_control, rtde_receive

if not USE_CONTROLLER:
    import keyboard 


# Keyboard control directions and commands
KEY_XM = 'f' #'s'
KEY_XP = 's' #'f'
KEY_YM = 'e' #'d'
KEY_YP = 'd' #'e'

KEY_ZP = 'i'
KEY_ZM = 'k'

KEY_PITCHP = 'u'
KEY_PITCHM = 'o'
KEY_ROLLP = 'j'
KEY_ROLLM = 'l'
KEY_YAWP = 'm'
KEY_YAWM = '.'

KEY_SPEEDP = 't'
KEY_SPEEDM = 'g'
KEY_ANGSPEEDP = 'y'
KEY_ANGSPEEDM = 'h'

KEY_QUIT = 'q'

## POSITION CONTROL
# Set position increments
INC_DELTA_PLANE = 0.01
INC_DELTA_HEIGHT = 0.01
INC_DELTA_ROT = 0.01

SPEED_L = 0.1 #1.0 #3.0 #0.5 #0.25
SPEED_L_MAX = 0.4
SPEED_ANG = 0.2 #0.1
SPEED_ANG_MAX = 0.4

SPEED_J = 1.05 #default speed and acceleration for joins
ACCEL_J = 1.4

ACCEL_L = 1.0 #0.1 #25
ACCEL_L_STOP = 10


## SPEED CONTROL
SPEED_STEP_PLANE = 0.1
SPEED_STEP_VERT = 0.1
SPEED_STEP_ROT = 0.1

LOOP_SLEEP_TIME = 0.1 # Run at 10 Hz

Q_HOME = [math.pi/2, -60.0*(math.pi/180.0), 40.0*(math.pi/180.0), -50*(math.pi/180.0), -90.0*(math.pi/180.0), 0.0]#[90.0, -60.0, 40.0, -50, -90.0, 0.0] # Home position in deg


def setup():
    # Connect to the robot
    if USE_ROBOT:
        # Connect to the robot
        rtde_c =  rtde_control.RTDEControlInterface(IP_UR5)
        rtde_r = rtde_receive.RTDEReceiveInterface(IP_UR5)

    else:
        rtde_c = None
        rtde_r = None

    # Connect to the gripper/ESP32 through serial interface
    # if USE_GRIPPER:

    # else: 
    #     pass

    # Setup the ps4 controller
    joystick = ps4.controller_init()

    print('Setup complete. Robot connected.')

    return rtde_c, rtde_r, joystick

## KEYBOARD CONTROL
# Alters the setpoint (either position or speed) based on the mode of control
def alter_setpoint(setpoint, ind, use_speed_control, speed, increment):
    new_setpoint = setpoint # Note this doesn't copy yet, only creates an alias
    
    if use_speed_control:
        new_setpoint[ind] = speed
    else:
        new_setpoint[ind] += increment

    return new_setpoint


def alter_setpoint_vel(speed, increment, ind, use_speed_control, delta_setpoint_vel, delta_increment_vel):
    new_speed = speed # Note this doesn't copy yet, only creates an alias
    new_increment = increment
    
    # Make speed increment, do not drop speed below 0 (inverts controls which is unintuitive)
    if use_speed_control:
        new_speed[ind] += delta_setpoint_vel
        new_speed[ind] = max(new_speed[ind], 0.0)
    else:
        new_increment[ind] += delta_increment_vel
        new_speed[ind] = max(new_speed[ind], 0.0)

    return new_speed, new_increment


# Poll the keyboard and return changes to the desired setpoints
def poll_keyboard(original_setpoint, use_speed_control, speed, increment):
    new_setpoint = original_setpoint # Note this doesn't copy yet, only creates an alias
    new_speed = speed
    new_increment = increment
    
    if keyboard.is_pressed(KEY_XM):
        new_setpoint = alter_setpoint(new_setpoint, 0, use_speed_control, -speed[0], -increment[0])

    elif keyboard.is_pressed(KEY_XP):
        new_setpoint = alter_setpoint(new_setpoint, 0, use_speed_control, speed[0], increment[0])

    if keyboard.is_pressed(KEY_YP):
        new_setpoint = alter_setpoint(new_setpoint, 1, use_speed_control, speed[0], increment[0])

    elif keyboard.is_pressed(KEY_YM):
        new_setpoint = alter_setpoint(new_setpoint, 1, use_speed_control, -speed[0], -increment[0])

    if keyboard.is_pressed(KEY_ZP):
        new_setpoint = alter_setpoint(new_setpoint, 2, use_speed_control, speed[1], increment[1])

    elif keyboard.is_pressed(KEY_ZM):
        new_setpoint = alter_setpoint(new_setpoint, 2, use_speed_control, -speed[1], -increment[1])

    if keyboard.is_pressed(KEY_PITCHP):
        new_setpoint = alter_setpoint(new_setpoint, 3, use_speed_control, speed[2], increment[2])

    elif keyboard.is_pressed(KEY_PITCHM):
        new_setpoint = alter_setpoint(new_setpoint, 3, use_speed_control, -speed[2], -increment[2])

    if keyboard.is_pressed(KEY_ROLLP):
        new_setpoint = alter_setpoint(new_setpoint, 4, use_speed_control, speed[2], increment[2])

    elif keyboard.is_pressed(KEY_ROLLM):
        new_setpoint = alter_setpoint(new_setpoint, 4, use_speed_control, -speed[2], -increment[2])

    if keyboard.is_pressed(KEY_YAWP):
        new_setpoint = alter_setpoint(new_setpoint, 5, use_speed_control, speed[2], increment[2])

    elif keyboard.is_pressed(KEY_YAWM):
        new_setpoint = alter_setpoint(new_setpoint, 5, use_speed_control, -speed[2], -increment[2])

    if keyboard.is_pressed(KEY_SPEEDP):
        new_speed, new_increment = alter_setpoint_vel(new_speed, new_increment, 0, use_speed_control, SPEED_STEP_PLANE, INC_DELTA_PLANE)
        new_speed, new_increment = alter_setpoint_vel(new_speed, new_increment, 1, use_speed_control, SPEED_STEP_PLANE, INC_DELTA_PLANE)
        
    elif keyboard.is_pressed(KEY_SPEEDM):
        new_speed, new_increment = alter_setpoint_vel(new_speed, new_increment, 0, use_speed_control, -SPEED_STEP_PLANE, -INC_DELTA_PLANE)
        new_speed, new_increment = alter_setpoint_vel(new_speed, new_increment, 1, use_speed_control, -SPEED_STEP_PLANE, -INC_DELTA_PLANE)

    if keyboard.is_pressed(KEY_ANGSPEEDP):
        new_speed, new_increment = alter_setpoint_vel(new_speed, new_increment, 2, use_speed_control, SPEED_STEP_ROT, INC_DELTA_ROT)
    
    elif keyboard.is_pressed(KEY_ANGSPEEDM):
        new_speed, new_increment = alter_setpoint_vel(new_speed, new_increment, 2, use_speed_control, -SPEED_STEP_ROT, -INC_DELTA_ROT)
    
    if keyboard.is_pressed(KEY_QUIT): 
        print('Quit key pressed')
        new_setpoint = None  # finishing the loop

    return new_setpoint, new_speed, new_increment

## CONTROL SPEED AND POSITION
def loop_speed_cntrl(rtde_c, joystick):
    speed = [SPEED_L, SPEED_L, SPEED_ANG] # plane, vertical, rotational
    increment = [0.0, 0.0, 0.0]
    #current_speedL_d = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    # Speed control loop
    while True:
        # Poll keyboard for speed direction and any speed setpoint changes
        current_speedL_d = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #TODO: speedStop(double a = 10.0)?? Stop arm overshooting, stopJ, stopL(double a = 10.0, bool asynchronous = false)
        #current_speedL_d, speed, increment = poll_keyboard(current_speedL_d, True, speed, increment)
        current_speedL_d, speedButtons = ps4.get_controller_input_scaled(joystick, SPEED_L_MAX, speed[2]) #SPEED_ANG_MAX
        
        if current_speedL_d is None:
            break
        
        # Adjust angular speed
        if speedButtons[0] == 1:
            speed[2] -= SPEED_STEP_ROT
            speed[2] = max(speed[2], 0.0)
        elif speedButtons[1] == 1:
            speed[2] += SPEED_STEP_ROT
            speed[2] = min(speed[2], SPEED_ANG_MAX)

        # Send speed command (or stop) to robot
        if USE_ROBOT:
            # Decelerate faster when stopping
            if all(v == 0 for v in current_speedL_d):
                rtde_c.speedStop(ACCEL_L_STOP)
            else:
                rtde_c.speedL(current_speedL_d, ACCEL_L, 0.1)

        # Send open/close or electromagnet on/off command to gripper
        # if USE_GRIPPER:
        #     pass

        print(current_speedL_d)

        time.sleep(LOOP_SLEEP_TIME) # Run at X Hz


def loop_pos_cntrl(rtde_c, rtde_r):
    ## POLLING LOOP
    speed = [0.0, 0.0, 0.0]
    increment_pos = [0.01, 0.01, 0.01] # plane, vertical, rotational increment sizes
    current_poseL_d = rtde_r.getActualTCPPose()
    
    while True:
        current_poseL_d, speed, increment_pos = poll_keyboard(current_poseL_d, False, speed, increment_pos)

        if current_poseL_d is None:
            break

        # Move to desired setpoint
        # Move asynchronously in cartesian space to target, we specify asynchronous behavior by setting the async parameter to
        # 'True'. Try to set the async parameter to 'False' to observe a default synchronous movement, which cannot be stopped
        # by the stopL function due to the blocking behaviour. Using 'True' makes it choppy as it sends more moveL commands every time without waiting
        rtde_c.moveL(current_poseL_d, SPEED_L, ACCEL_L, False)
        print(current_poseL_d)

        time.sleep(LOOP_SLEEP_TIME) # Run at X Hz
        
#move to home position (no singularity)
def move_home(q_desired, speed, acceleration, asynchronous=False):
    rtde_c.moveJ(q_desired, speed, acceleration, asynchronous)
    return


if __name__ == "__main__":
    # SETUP
    rtde_c, rtde_r, joystick = setup()

    if USE_ROBOT:
        move_home(Q_HOME, SPEED_J, ACCEL_J, False)

    # LOOP
    print('About to enter manual control loop. Press q to quit.')
    #loop_pos_cntrl(rtde_c, rtde_r)
    loop_speed_cntrl(rtde_c, joystick)





# TODO: prevent going into singularity, OR: if robot is in singularity, get out!!!!!
# Home position -> only go to home when press button
# Allow joint control through keyboard??
# Perhaps change the orientation to just wrist control?? And and x-y-z just to wrist-3 

# Have button that goes down to particular height to pick tim up, then up to a particular height to allow movement


# Perhaps: move asynchronously parallel to windows of rubble until high IR detected and stop there? or store highest IR value and move back to that

# USEFUL FUNCTIONS: 
# rtde_c.moveUntilContact(speed)
# moveJ - in joint space, moveL - in cartesian space
# moveJ_IK - specify a pose that a robot will move to linearly in joint space
# ***moveL - can set pos + speed + acceleration. Moves linearly in tool space
#       - can specify "path" - 2d vector of doubles that also includes speeds and accelerations
# ***movePath - move to each waypoint specified in a given path   