import time
import PS4_Control as ps4
import math

# PARAMETERS
PRINT_SPEED = False

# Robot IP address
IP_UR5 = "169.254.157.1" #"192.168.0.88" #"169.254.157.0"

# Components
USE_ROBOT = True #True #True
USE_CONTROLLER = True
USE_GRIPPER = False #True 

if USE_ROBOT:
    import rtde_control, rtde_receive

if not USE_CONTROLLER:
    import keyboard 

if USE_GRIPPER:
    import serial
    import time

# Serial configuration for gripper
port = '/dev/ttyACM0' 
baud_rate = 115200
timeout = 2  # Timeout for serial communication

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

CLAW_OPEN = "v" # 'share' toggles gripper
CLAW_CLOSE = "c"
MAG_ON = "m" # 'options' toggles EM
MAG_OFF = "n"

## POSITION CONTROL
# Set position increments
INC_DELTA_PLANE = 0.01
INC_DELTA_HEIGHT = 0.01
INC_DELTA_ROT = 0.01

SPEED_L = 0.1 #1.0 #3.0 #0.5 #0.25
SPEED_L_MAX = 0.2 #0.4
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

Q_HOME = [math.pi/2, -60.0*(math.pi/180.0), 40.0*(math.pi/180.0), -70*(math.pi/180.0), -90.0*(math.pi/180.0), 0.0] # Home position in deg
#[-60.0*(math.pi/180.0), -93.0*(math.pi/180.0), -71.0*(math.pi/180.0), -104.0*(math.pi/180.0), 90.0*(math.pi/180.0), 14.0*(math.pi/180.0)]

True
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
    if USE_GRIPPER:
        # Initialize serial connection
        try:
            gripper_serial = serial.Serial(port, baud_rate, timeout=timeout)
            print("Connected to gripper on port " + port)
        except Exception as e:
            print("Failed to connect gripper on port: " + str(port))
            print(str(e))
            exit()
    else:
        gripper_serial = None
    # Setup the ps4 controller
    if USE_CONTROLLER:
        joystick = ps4.controller_init()
    else:
        joystick = None

    print('Setup complete. Robot connected.')

    return rtde_c, rtde_r, joystick, gripper_serial

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
def loop_speed_cntrl(rtde_c, joystick, gripper_serial, rtde_r):
    # Tracking variables
    speed = [SPEED_L, SPEED_L, SPEED_ANG] # plane, vertical, rotational
    increment = [0.0, 0.0, 0.0]
    gripper_open = False # False = closed, True = open
    magnet_on = False # False = off, True = on
    #Roofs = ~34, 24.7
   
    # Speed control loop
    while True:
        theta = 0.0 # Ange in cylindrical coordinates
        
        if USE_ROBOT:
            # Get current pose and z position
            current_poseL_d = rtde_r.getActualTCPPose()
            offset_tim = 17.2 
            current_z_cm = round(current_poseL_d[2]*100,1) - offset_tim # z position for TIM, 0 = good position for picking up

            theta = math.atan2(current_poseL_d[0], -current_poseL_d[1]) #current_poseL_d[1], current_poseL_d[0]) # Angle in cylindrical coordinates TODO: TESTTTTTTTT

        
        # Poll keyboard for speed direction and any speed setpoint changes
        current_speedL_d = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #TODO: speedStop(double a = 10.0)?? Stop arm overshooting, stopJ, stopL(double a = 10.0, bool asynchronous = false)
        #current_speedL_d, speed, increment = poll_keyboard(current_speedL_d, True, speed, increment)
        current_speedL_d, speedButtons, toggle_gripper, toggle_magnet, reset_home, zPickUp = ps4.get_controller_input_scaled(joystick, SPEED_L_MAX, speed[2], theta) #SPEED_ANG_MAX

        if current_speedL_d is None:
            break
        
         # Send home/reset
        if reset_home:
            reset()

        # Adjust angular speed
        if speedButtons[0] == 1:
            speed[2] -= SPEED_STEP_ROT
            speed[2] = max(speed[2], 0.0)
        elif speedButtons[1] == 1:
            speed[2] += SPEED_STEP_ROT
            speed[2] = min(speed[2], SPEED_ANG_MAX)

        # Send speed command (or stop) to robot, and other commands
        if USE_ROBOT:
            # Get current pose and z position
            # current_poseL_d = rtde_r.getActualTCPPose()
            # offset_tim = 17.2 
            # current_z_cm = round(current_poseL_d[2]*100,1) - offset_tim # z position for TIM, 0 = good position for picking up
            
            # Decelerate faster when stopping
            if all(v == 0 for v in current_speedL_d):
                rtde_c.speedStop(ACCEL_L_STOP)
            else:
                rtde_c.speedL(current_speedL_d, ACCEL_L, 0.1)
            
            #SIGULARITY FUNCTION
            speed_time = [x * .5 for x in current_speedL_d] #multiply each speed value by time step
            new_pose = [z + y for z, y in zip(current_poseL_d, speed_time)] #create new list of positions
            sing = rtde_c.isPoseWithinSafetyLimits(new_pose) #false if future position is singularity
            if not sing: 
                rtde_c.speedStop() #STOP SPEED 
                print("SINGULARITY ALMOST HIT STOP")

            # If the z-pickup functionality is enabled
            if zPickUp:
                # single axis target
                target = offset_tim / 100 # Target Height
                eps = 0.005 # 5 mm epsilon 
                
                if (current_poseL_d[2] <= target + eps) and (current_poseL_d[2] >= target - eps):
                    try:
                        target = old_z_height
                    except: 
                        target = target + 0.25 
                else:
                    old_z_height = current_poseL_d[2]
               
                targetPickUp = [c if i!= 2 else target for i,c in enumerate(current_poseL_d)]
                rtde_c.moveL(targetPickUp, SPEED_L, ACCEL_L, False) #move down to good location above Tim
        if zPickUp:
                # single axis target
                target = offset_tim / 100 # Target Height
                eps = 0.005 # 5 mm epsilon 
                
                if (current_poseL_d[2] <= target + eps) and (current_poseL_d[2] >= target - eps):
                    try:
                        target = old_z_height
                    except: 
                        target = target + 0.25 
                else:
                    old_z_height = current_poseL_d[2]
               
                targetPickUp = [c if i!= 2 else target for i,c in enumerate(current_poseL_d)]
                rtde_c.moveL(targetPickUp, SPEED_L, ACCEL_L, False) #move down to good location above Tim
        
        # Send open/close or electromagnet on/off command to gripper
        if USE_GRIPPER:
            # Toggle gripper 
            if toggle_gripper:
                # Determine if the gripper should open or close
                cmd = CLAW_CLOSE
                
                if not gripper_open:
                    cmd = CLAW_OPEN

                # Send the command
                send_gripper_cmd(gripper_serial, cmd)
                gripper_open = not gripper_open   

            if toggle_magnet:
                # Determine if the magnet should turn on or off
                cmd = MAG_ON
                
                if magnet_on:
                    cmd = MAG_OFF

                # Send the command
                send_gripper_cmd(gripper_serial, cmd)
                magnet_on = not magnet_on

        # Print current speed
        if PRINT_SPEED:
            print(current_speedL_d)

            # Also print current z position
            if USE_ROBOT:
                print(current_z_cm) #z position of TIM, 0 = good position to pick up TIM

        time.sleep(LOOP_SLEEP_TIME) # Run at X Hz


def loop_pos_cntrl(rtde_c, rtde_r):
    ## POLLING LOOP
    speed = [0.0, 0.0, 0.0]
    increment_pos = [0.01, 0.01, 0.01] # plane, vertical, rotational increment sizes
    
    
    while True:
        current_poseL_d, speed, increment_pos = poll_keyboard(current_poseL_d, False, speed, increment_pos)

        if current_poseL_d is None:
            break

        # Move to desired setpoint
        # Move asynchronously in cartesian space to target, we specify asynchronous behavior by setting the async parameter to
        # 'True'. Try to set the async parameter to 'False' to observe a default synchronous movement, which cannot be stopped
        # by the stopL function due to the blocking behaviour. Using 'True' makes it choppy as it sends more moveL commands every time without waiting
        rtde_c.moveL(current_poseL_d, SPEED_L, ACCEL_L, False)

        if PRINT_SPEED:
            print(current_poseL_d)

        #     while True:
        # cmd = input("Enter command (OPEN (v), CLOSE (c), MAG_ON (m), MAG_OFF (n), or q to quit): ")
        # if cmd.lower() == 'q':
        #     break
        # elif cmd in ['v', 'c', 'm', 'n']:
        #     send_command(cmd)
        # else:
        #     print("Invalid command. Please try again.")

        time.sleep(LOOP_SLEEP_TIME) # Run at X Hz
        
#move to home position (no singularity)
def move_home(q_desired, speed, acceleration, asynchronous=False):
    rtde_c.moveJ(q_desired, speed, acceleration, asynchronous)
    return

 # Send a command to the ESP32 (which the gripper runs off) via serial
def send_gripper_cmd(gripper_serial, command):
    gripper_serial.write((command + '\n').encode())  # Command must end with a newline character
    time.sleep(0.1)  # Give some time for the ESP32 to respond
    while gripper_serial.in_waiting:
        print(gripper_serial.readline().decode().strip())  # Print the ESP32's response


def reset():
    print('RESET')

    # Reset the robot to home position
    if USE_ROBOT:
        move_home(Q_HOME, SPEED_J, ACCEL_J, False)

    # if USE_GRIPPER:
    #     send_gripper_cmd(gripper_serial, CLAW_CLOSE)
    #     send_gripper_cmd(gripper_serial, MAG_OFF)


if __name__ == "__main__":
    # SETUP
    rtde_c, rtde_r, joystick, gripper_serial = setup()
    #reset()

    # LOOP
    print('About to enter manual control loop. Press q to quit.')
    #loop_pos_cntrl(rtde_c, rtde_r)
    loop_speed_cntrl(rtde_c, joystick, gripper_serial, rtde_r)








# Perhaps: move asynchronously parallel to windows of rubble until high IR detected and stop there? or store highest IR value and move back to that

# USEFUL FUNCTIONS: 
# rtde_c.moveUntilContact(speed)
# moveJ - in joint space, moveL - in cartesian space
# moveJ_IK - specify a pose that a robot will move to linearly in joint space
# ***moveL - can set pos + speed + acceleration. Moves linearly in tool space
#       - can specify "path" - 2d vector of doubles that also includes speeds and accelerations
# ***movePath - move to each waypoint specified in a given path   