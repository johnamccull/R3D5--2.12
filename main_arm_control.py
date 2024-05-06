import time
import PS4_Control as ps4
import math
import keyboard

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

if USE_GRIPPER:
    import serial
    import time

# Serial configuration for gripper
port = '/dev/ttyACM0' 
baud_rate = 115200
timeout = 2  # Timeout for serial communication

# Keyboard control directions and commands
# KEY_XM = 'f' #'s'
# KEY_XP = 's' #'f'
# KEY_YM = 'e' #'d'
# KEY_YP = 'd' #'e'

# KEY_ZP = 'i'
# KEY_ZM = 'k'

# KEY_PITCHP = 'u'
# KEY_PITCHM = 'o'
# KEY_ROLLP = 'j'
# KEY_ROLLM = 'l'
# KEY_YAWP = 'm'
# KEY_YAWM = '.'

# KEY_SPEEDP = 't'
# KEY_SPEEDM = 'g'
# KEY_ANGSPEEDP = 'y'
# KEY_ANGSPEEDM = 'h'

KEY_QUIT = 'q'

KEY_ROOF1 = "1" #house 1 (middle)
KEY_ROOF2 = "2" #house 2 (right)
KEY_ROOF3 = "3" #house 3 (left)

KEY_ROOF_A = "a" #house 1 orientation 1
KEY_ROOF_B = "b"
KEY_ROOF_C = "c" #house 1 orientation 2
KEY_ROOF_D = "d"
KEY_ROOF_E = "e" #house 2 orientation 1
KEY_ROOF_F = "f"
KEY_ROOF_G = "g" #house 2 orientation 2
KEY_ROOF_H = "h"
KEY_ROOF_I = "i" #house 3 orientation 1
KEY_ROOF_J = "j"
KEY_ROOF_K = "k" #house 3 orientation 2
KEY_ROOF_L = "l"

KEY_PICKUP_ROOF = "0" #go to roof and pickup 

## Tool Position of roofs for each house for pickup
th = 1000
add = 0.005
POS_A = [451/th, 329.5/th, 26/th + add, 3.134, 0.187, 0.061]
POS_B = [417.6/th, 585.2/th, 128/th + add, 2.827, 1.388, 0.046]
POS_C = [406.85/th, 205.4/th, 128.2/th + add, 0.884, 3.036, 0.011]
POS_D = [406.03/th, 458.5/th, 27.39/th + add, 0.074, -3.117, -0.015]
POS_E = [352.2/th, -445.3/th, 26/th + add, 3.040, -0.736, 0.060]
POS_F = [622.5/th, -387.8/th, 129.7/th + add, 3.11, -0.292, 0.094]
POS_G = [219.8/th, -469.1/th, 128.5/th + add, 2.559, -1.757, 0.042]
POS_H = [485.3/th, -395.3/th, 27.9/th + add, 2.899, -1.045, 0.057]
POS_I = [-297.4/th, 485.3/th, 24.6/th + add, 2.351, 2.109, 0.041]
POS_J = [-528.5/th, 371.9/th, 124.3/th + add, 0.434, 3.17, 0.003]
POS_K = [-293.6/th, 471.1/th, 126.1/th + add, 2.612, 1.779, 0.068]
POS_L = [-503.4/th, 424.7/th, 24.5/th + add, 2.489, 1.962, 0.029]
# Positions of each house for heating
POS_1 = [465.8/th, 457.34/th, 196.9/th, 2.968, 1.106, 0.045]
POS_2 = [410.6/th, -458.1/th, 134.3/th, 2.594, -1.885, 0.011]
POS_3 = [-381.7/th, 452.5/th, 142.2/th, 1.085, 3.023, -0.202]


CLAW_OPEN = "v" # 'share' toggles gripper
CLAW_CLOSE = "x"
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
    
    # if keyboard.is_pressed(KEY_XM):
    #     new_setpoint = alter_setpoint(new_setpoint, 0, use_speed_control, -speed[0], -increment[0])

    # elif keyboard.is_pressed(KEY_XP):
    #     new_setpoint = alter_setpoint(new_setpoint, 0, use_speed_control, speed[0], increment[0])

    # if keyboard.is_pressed(KEY_YP):
    #     new_setpoint = alter_setpoint(new_setpoint, 1, use_speed_control, speed[0], increment[0])

    # elif keyboard.is_pressed(KEY_YM):
    #     new_setpoint = alter_setpoint(new_setpoint, 1, use_speed_control, -speed[0], -increment[0])

    # if keyboard.is_pressed(KEY_ZP):
    #     new_setpoint = alter_setpoint(new_setpoint, 2, use_speed_control, speed[1], increment[1])

    # elif keyboard.is_pressed(KEY_ZM):
    #     new_setpoint = alter_setpoint(new_setpoint, 2, use_speed_control, -speed[1], -increment[1])

    # if keyboard.is_pressed(KEY_PITCHP):
    #     new_setpoint = alter_setpoint(new_setpoint, 3, use_speed_control, speed[2], increment[2])

    # elif keyboard.is_pressed(KEY_PITCHM):
    #     new_setpoint = alter_setpoint(new_setpoint, 3, use_speed_control, -speed[2], -increment[2])

    # if keyboard.is_pressed(KEY_ROLLP):
    #     new_setpoint = alter_setpoint(new_setpoint, 4, use_speed_control, speed[2], increment[2])

    # elif keyboard.is_pressed(KEY_ROLLM):
    #     new_setpoint = alter_setpoint(new_setpoint, 4, use_speed_control, -speed[2], -increment[2])

    # if keyboard.is_pressed(KEY_YAWP):
    #     new_setpoint = alter_setpoint(new_setpoint, 5, use_speed_control, speed[2], increment[2])

    # elif keyboard.is_pressed(KEY_YAWM):
    #     new_setpoint = alter_setpoint(new_setpoint, 5, use_speed_control, -speed[2], -increment[2])

    # if keyboard.is_pressed(KEY_SPEEDP):
    #     new_speed, new_increment = alter_setpoint_vel(new_speed, new_increment, 0, use_speed_control, SPEED_STEP_PLANE, INC_DELTA_PLANE)
    #     new_speed, new_increment = alter_setpoint_vel(new_speed, new_increment, 1, use_speed_control, SPEED_STEP_PLANE, INC_DELTA_PLANE)
        
    # elif keyboard.is_pressed(KEY_SPEEDM):
    #     new_speed, new_increment = alter_setpoint_vel(new_speed, new_increment, 0, use_speed_control, -SPEED_STEP_PLANE, -INC_DELTA_PLANE)
    #     new_speed, new_increment = alter_setpoint_vel(new_speed, new_increment, 1, use_speed_control, -SPEED_STEP_PLANE, -INC_DELTA_PLANE)

    # if keyboard.is_pressed(KEY_ANGSPEEDP):
    #     new_speed, new_increment = alter_setpoint_vel(new_speed, new_increment, 2, use_speed_control, SPEED_STEP_ROT, INC_DELTA_ROT)
    
    # elif keyboard.is_pressed(KEY_ANGSPEEDM):
    #     new_speed, new_increment = alter_setpoint_vel(new_speed, new_increment, 2, use_speed_control, -SPEED_STEP_ROT, -INC_DELTA_ROT)
    
    if keyboard.is_pressed(KEY_ROOF1):
        rtde_c.moveL(POS_1, SPEED_J, ACCEL_J, False)

    elif keyboard.is_pressed(KEY_ROOF2):
        rtde_c.moveL(POS_2, SPEED_J, ACCEL_J, False)
    
    if keyboard.is_pressed(KEY_ROOF3):
        rtde_c.moveL(POS_3, SPEED_J, ACCEL_J, False)

    elif keyboard.is_pressed(KEY_ROOF_A):
        rtde_c.moveL(POS_A, SPEED_J, ACCEL_J, False)

    if keyboard.is_pressed(KEY_ROOF_B):
        rtde_c.moveL(POS_B, SPEED_J, ACCEL_J, False)
    
    elif keyboard.is_pressed(KEY_ROOF_C):
        rtde_c.moveL(POS_C, SPEED_J, ACCEL_J, False)
    
    if keyboard.is_pressed(KEY_ROOF_D):
        rtde_c.moveL(POS_D, SPEED_J, ACCEL_J, False)

    elif keyboard.is_pressed(KEY_ROOF_E):
        rtde_c.moveL(POS_E, SPEED_J, ACCEL_J, False)
    
    if keyboard.is_pressed(KEY_ROOF_F):
        rtde_c.moveL(POS_F, SPEED_J, ACCEL_J, False)

    if keyboard.is_pressed(KEY_ROOF_G):
        rtde_c.moveL(POS_G, SPEED_J, ACCEL_J, False)

    if keyboard.is_pressed(KEY_ROOF_H):
        rtde_c.moveL(POS_H, SPEED_J, ACCEL_J, False)

    if keyboard.is_pressed(KEY_ROOF_I):
        rtde_c.moveL(POS_I, SPEED_J, ACCEL_J, False)

    if keyboard.is_pressed(KEY_ROOF_J):
        rtde_c.moveL(POS_J, SPEED_J, ACCEL_J, False)

    if keyboard.is_pressed(KEY_ROOF_K):
        rtde_c.moveL(POS_K, SPEED_J, ACCEL_J, False)

    if keyboard.is_pressed(KEY_ROOF_L):
        rtde_c.moveL(POS_L, SPEED_J, ACCEL_J, False)
    

    if keyboard.is_pressed(KEY_PICKUP_ROOF):
        current_poseL_d = rtde_r.getActualTCPPose()
        def pick_up_roof():
            send_gripper_cmd(gripper_serial, MAG_ON)
            time.sleep(100)
            eps = 0.0005 #0.5 mm epsilon
            force_target = 10 #amount of force to push down on the roof with
            forces = rtde_r.actual_TCP_force()
            z_force = forces[2]
            if (z_force <= force_target + eps) and (z_force >= force_target - eps):
                 try:
                     z_move = old_z
                 except: 
                     z_move = z_move + 0.05 
                 else:
                     old_z = current_poseL_d[2]

            tofTargetPickUp = [c if i!= 2 else z_move for i,c in enumerate(current_poseL_d)]
            rtde_c.moveL(tofTargetPickUp, SPEED_L, ACCEL_L, False) #move down to good location above roof
            time.sleep(100) #before moving to next position 

            z_move = z_move + 0.05
            moveUpPosition = [c if i!= 2 else z_move for i,c in enumerate(current_poseL_d)]
            rtde_c.moveL(moveUpPosition, SPEED_L, ACCEL_L, False) #move up 
            trashPos = [-0.3911, 0.2967, 0.6767, -0.0065, -3.1348, 0.0010] #trash pos, out of the way of everything
            rtde_c.moveL(trashPos,SPEED_L, ACCEL_L, False) #moce to trash position
            time.sleep(100)
            send_gripper_cmd(gripper_serial, MAG_OFF) #turn off magnet 

        # def pick_up_roof():
        #     send_gripper_cmd(gripper_serial, MAG_ON)
        #     time.sleep(100)
        #     tofValue = #value taken from tof sensor
        #     tofTarget = 0.015 #15 mm from ToF to base of magnet (measure exact value with calipers soon)
        #     eps = 0.0005 #0.5 mm epsilon 
        #     if (tofValue <= tofTarget + eps) and (tofValue >= tofTarget - eps):
        #         try:
        #             tofTarget = old_z
        #         except: 
        #             tofTarget = tofTarget + 0.05 
        #         else:
        #             old_z = current_poseL_d[2]
        #     tofTargetPickUp = [c if i!= 2 else tofTarget for i,c in enumerate(current_poseL_d)]
        #     rtde_c.moveL(tofTargetPickUp, SPEED_L, ACCEL_L, False) #move down to good location above roof
        #     time.sleep(100) #before moving to next position 

        #     tofTargetUp = tofTarget + 0.05
        #     moveUpPosition = [c if i!= 2 else tofTargetUp for i,c in enumerate(current_poseL_d)]
        #     rtde_c.moveL(moveUpPosition, SPEED_L, ACCEL_L, False) #move up 
        #     trashPos = [-0.3911, 0.2967, 0.6767, -0.0065, -3.1348, 0.0010] #trash pos, out of the way of everything
        #     rtde_c.moveL(trashPos,SPEED_L, ACCEL_L, False) #moce to trash position
        #     time.sleep(100)
        #     send_gripper_cmd(gripper_serial, MAG_OFF) #turn off magnet 

    elif keyboard.is_pressed(KEY_QUIT): 
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
    theta = 0.0 # Ange in cylindrical coordinates
    
    if USE_ROBOT:
        # Get current pose and z position
        current_poseL_d = rtde_r.getActualTCPPose()
        offset_tim = 17.2 
        current_z_cm = round(current_poseL_d[2]*100,1) - offset_tim # z position for TIM, 0 = good position for picking up
        forces = rtde_r.getActualTCPForce()
        print(forces[2])

        theta = math.atan2(current_poseL_d[0], -current_poseL_d[1]) #current_poseL_d[1], current_poseL_d[0]) # Angle in cylindrical coordinates TODO: TESTTTTTTTT

    
    # Poll keyboard for speed direction and any speed setpoint changes
    current_speedL_d = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #TODO: speedStop(double a = 10.0)?? Stop arm overshooting, stopJ, stopL(double a = 10.0, bool asynchronous = false)
    #current_speedL_d, speed, increment = poll_keyboard(current_speedL_d, True, speed, increment)
    current_speedL_d, speedButtons, toggle_gripper, toggle_magnet, reset_home, zPickUp = ps4.get_controller_input_scaled(joystick, SPEED_L_MAX, speed[2], theta) #SPEED_ANG_MAX
    new_setpoint, new_speed, new_increment,  = poll_keyboard()
    
    while True:
        if current_speedL_d is None:
        #     break
        
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

    current_poseL_d = rtde_r.getActualTCPPose()
    current_poseL_d, speed, increment_pos = poll_keyboard(current_poseL_d, False, speed, increment_pos)

    while True:
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


def pick_up_roof():

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
    # while True:
    #     loop_pos_cntrl(rtde_c, rtde_r)
    loop_speed_cntrl(rtde_c, joystick, gripper_serial, rtde_r)







# Perhaps: move asynchronously parallel to windows of rubble until high IR detected and stop there? or store highest IR value and move back to that

# USEFUL FUNCTIONS: 
# rtde_c.moveUntilContact(speed)
# moveJ - in joint space, moveL - in cartesian space
# moveJ_IK - specify a pose that a robot will move to linearly in joint space
# ***moveL - can set pos + speed + acceleration. Moves linearly in tool space
#       - can specify "path" - 2d vector of doubles that also includes speeds and accelerations
# ***movePath - move to each waypoint specified in a given path   
