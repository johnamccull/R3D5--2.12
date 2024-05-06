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
USE_GRIPPER = True #True 

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

## Tool Position of roofs for each house for pickup
th = 1000
add = 0.005
POS_A = [0.3958538304842751, 0.24978578871982376, 0.4265666168884772, 1.9883451087942299, -2.3977435469339814, 0.02286264994955303]
POS_B = [0.4092870942274119, 0.5483542469617233, 0.5286425117253378, 1.9883692388655774, -2.3978177916856684, 0.022894572212964332]
POS_C = [0.4030707912776921, 0.215665444427723, 0.526003723875395, 1.9883499113339607, -2.397823187159072, 0.02290414677337086]
POS_D = [0.4101288759544432, 0.5265357593222997, 0.427156556531685, 1.9883333272460297, -2.397858800931703, 0.022896279332703894]
POS_E = [0.2783081094708031, -0.41421618465988597, 0.42666308407224873, -0.6639307093768754, -3.0420190252667583, -0.0184182392018774]
POS_F = [0.5717924355794337, -0.40274239600216555, 0.5274792323051698, -0.6639571879461229, -3.041925912960904, -0.018492578021733562]
POS_G = [0.21054452841668214, -0.4331911083263245, 0.5292614231507237, -0.6640240853685172, -3.04198668923551, -0.01844375440366032]
POS_H = [0.5090242631607035, -0.4272351123624579, 0.4237870999921936, -0.6639618900279403, -3.0420488352586936, -0.01830644974883643]
POS_I = [-0.40905557951236143, -0.2809979625439197, 0.42713349578122217, 1.9538697848188273, -2.4156933350086414, 0.03736850358701248]
POS_J = [-0.3543289143751489, -0.5939681337677466, 0.5294571649322793, 1.9575821072090849, -2.4201960483287888, 0.026043673931247388]
POS_K = [-0.43194868345476406, -0.19660842623538524, 0.5275326549104137, 1.9884084854752329, -2.3978243461975137, 0.022944633757596065]
POS_L = [-0.4266824187990573, -0.5165109943271625, 0.42725743040494474, 1.9538771314063674, -2.415675978403866, 0.0373408386426631]
# Positions of each house for heating
POS_1 = [0.4359194818885224, 0.36427965567375403, 0.533283094482392, -0.6639689819910086, -3.0420612002505196, -0.0183648934540018]
POS_2 = [0.40592256671743415, -0.4132648902196437, 0.554846955641282, -2.812422348609393, -1.380806605309295, -0.04573916793366082]
POS_3 = [-0.4212487453556083, -0.3790010411135084, 0.5311406155165554, 1.9883662349156, -2.3977572483191896, 0.022869172731757304]


key_roofs = {'a':POS_A,'b':POS_B, 'c':POS_C, 'd':POS_D, 'e':POS_E, 'f':POS_F, 'g':POS_G, 
                  'h':POS_H, 'i': POS_I, 'j':POS_J,'k': POS_K, 'l':POS_L} #maps keys to positions of roofs to lift and throw them away

key_houses = {'1': POS_1, '2':POS_2, '3':POS_3} #maps keys to positions of houses to IR Sense

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
def poll_keyboard():
    
    # key = keyboard.read_key()
    for key in key_houses:
        if keyboard.is_pressed(key):
            moveToRoof(key_houses[key])
    
    for key in key_roofs: 
        if keyboard.is_pressed(key):
            roofTrash(key_roofs[key])

    return 

## CONTROL SPEED AND POSITION 

def loop_speed_cntrl(rtde_c, joystick, gripper_serial, rtde_r):
    # Tracking variables
    speed = [SPEED_L, SPEED_L, SPEED_ANG] # plane, vertical, rotational
    increment = [0.0, 0.0, 0.0]
    gripper_open = False # False = closed, True = open
    magnet_on = False # False = off, True = on

    while True: 
    # Speed control loop
        theta = 0.0 # Ange in cylindrical coordinates

        if USE_ROBOT:
            # Get current pose and z position
            current_poseL_d = rtde_r.getActualTCPPose()
            offset_tim = 17.2 
            current_z_cm = round(current_poseL_d[2]*100,1) - offset_tim # z position for TIM, 0 = good position for picking up

            theta = math.atan2(current_poseL_d[0], -current_poseL_d[1]) #current_poseL_d[1], current_poseL_d[0]) # Angle in cylindrical coordinates TODO: TESTTTTTTTT

        # Poll keyboard for speed direction and any speed setpoint changes
        current_speedL_d = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #TODO: speedStop(double a = 10.0)?? Stop arm overshooting, stopJ, stopL(double a = 10.0, bool asynchronous = false)
        current_speedL_d, speedButtons, toggle_gripper, toggle_magnet, reset_home, zPickUp = ps4.get_controller_input_scaled(joystick, SPEED_L_MAX, speed[2], theta) #SPEED_ANG_MAX

        if not all(current_speedL_d):
            poll_keyboard()
        
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


def roofTrash(position):
    rtde_c.moveL(position, SPEED_L, ACCEL_L, False)
    time.sleep(1)
    current_poseL_d = rtde_r.getActualTCPPose()
    send_gripper_cmd(gripper_serial, MAG_ON)
    time.sleep(1)
    z_move = position[2] + 0.12
    tofTargetPickUp = [c if i!= 2 else z_move for i,c in enumerate(current_poseL_d)]
    rtde_c.moveL(tofTargetPickUp, SPEED_L, ACCEL_L, False) #move down to good location above roof
    time.sleep(1) #before moving to next position
    # trashPos = [-0.3212384054551387, 0.3778070867813049, 0.676713272187646, -0.006513678852313608, -3.1347676475344275, 0.0008835774462710534] #trash pos, out of the way of everything
    # rtde_c.moveL(trashPos,SPEED_L, ACCEL_L, False) #move to trash position
    # time.sleep(1)
    # send_gripper_cmd(gripper_serial, MAG_OFF) #turn off magnet 

def moveToRoof(position): 
    rtde_c.moveL(position, SPEED_L, ACCEL_L, False)
    
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
