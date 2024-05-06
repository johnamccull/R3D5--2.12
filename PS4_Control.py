import pygame
import numpy as np

PRINT = False
MAPPING_LINUX = True # Harvey's controller key mappings are different to Baran's. Switch here/DO NOT JUST CHANGE THE BUTTONS BELOW

CS_CYLINDRICAL = True

def smooth_data(data, threshold=0.1):
    """
    Smooths joystick data and removes drift.

    Args:
    - data (list): List of joystick axis values
    - threshold (float): Threshold for drift removal

    Returns:
    - Smoothed data (list)
    """
    smoothed_data = []
    for value in data:
        if abs(value) < threshold:
            smoothed_data.append(0.0)
        else:
            smoothed_data.append(value)
    return smoothed_data

def rotation_matrix_roll(angle):
    """
    Generate a rotation matrix for roll (rotation around x-axis).

    Args:
    - angle (float): Angle of rotation in radians.

    Returns:
    - Rotation matrix (numpy array).
    """
    return np.array([[1, 0, 0],
                     [0, np.cos(angle), -np.sin(angle)],
                     [0, np.sin(angle), np.cos(angle)]])

def rotation_matrix_pitch(angle):
    """
    Generate a rotation matrix for pitch (rotation around y-axis).

    Args:
    - angle (float): Angle of rotation in radians.

    Returns:
    - Rotation matrix (numpy array).
    """
    return np.array([[np.cos(angle), 0, np.sin(angle)],
                     [0, 1, 0],
                     [-np.sin(angle), 0, np.cos(angle)]])

def rotation_matrix_yaw(angle):
    """
    Generate a rotation matrix for yaw (rotation around z-axis).

    Args:
    - angle (float): Angle of rotation in radians.

    Returns:
    - Rotation matrix (numpy array).
    """
    return np.array([[np.cos(angle), -np.sin(angle), 0],
                     [np.sin(angle), np.cos(angle), 0],
                     [0, 0, 1]])

def coord_transform(roll, pitch, yaw, R):
    """
    Perform coordinate transform from tool to base coordinates.

    Args:
    - roll,pitch,yaw (float): Anglular velocity of rotation in radians/sec.

    Returns:
    - Rotation matrix (numpy array).
    """
    R_rol = 0#get roll
    R_pit = 0#get pitch
    R_yaw = 0#get yaw
    roll_matrix = rotation_matrix_roll(R_rol)
    pitch_matrix = rotation_matrix_pitch(R_pit)
    yaw_matrix = rotation_matrix_yaw(R_yaw)

    R_e = np.matmul(yaw_matrix, np.matmul(pitch_matrix, roll_matrix))

    omega_dot_e = np.array([[roll],
                            [pitch],
                            [yaw]])
    
    omega_dot_0 = np.matmul(R_e,omega_dot_e)
    return omega_dot_0


# Initialize ps4 controller
def controller_init():
    # Initialize pygame
    pygame.init()
    pygame.joystick.init()

    # Check for connected joysticks
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("No joystick connected.")
        quit()

    # Get the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    return joystick


# Get input from the PS4 controller
def get_controller_input(joystick, arm_theta):
    # Main loop
    # running = True
    # while running:

    # Check for events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Get joystick axes and buttons
    axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
    axes = smooth_data(axes)
    buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]

    # Print axes and buttons
    if PRINT:
        print("Axes:", axes)
        print("Buttons:", buttons)

    #Group to PS4 Layout
    if MAPPING_LINUX:
        # Right Button Pad
        tri_but = buttons[2] #buttons[3]
        sq_but = buttons[3] #buttons[2]
        cir_but = buttons[1]
        x_but = buttons[0]
        #up_but = buttons[11]
        #l_but = buttons[13]
        #r_but = buttons[14]
        #d_but = buttons[12]
        rb_pad = [tri_but,sq_but,cir_but,x_but]
        #lb_pad = [up_but,l_but,r_but,d_but]
        options = buttons[9] 
        share = buttons[8]  
        home = buttons[10]
        touchpad = buttons[len(buttons)-1]
        r1 = buttons[5] #buttons[10]
        r2 = buttons[7] #axes[len(axes)-1]
        l1 = buttons[4] #buttons[9]
        l2 = buttons[6] #axes[len(axes)-2]
        rl_buttons = [r1,l1,r2,l2]
        l_pad = [axes[0],axes[1],buttons[11]] #buttons[7]
        r_pad = [axes[3],axes[4],buttons[12]] #axes[2] axes[3] buttons[8]
    else:
        # Right Button Pad
        tri_but = buttons[2] #buttons[3]
        sq_but = buttons[3] #buttons[2]
        cir_but = buttons[1]
        x_but = buttons[0]
        #up_but = buttons[11]
        #l_but = buttons[13]
        #r_but = buttons[14]
        #d_but = buttons[12]
        rb_pad = [tri_but,sq_but,cir_but,x_but]
        #lb_pad = [up_but,l_but,r_but,d_but]
        options = buttons[6] 
        share = buttons[4]  
        touchpad = buttons[len(buttons)-1]
        r1 = buttons[5] #buttons[10]
        r2 = buttons[7] #axes[len(axes)-1]
        l1 = buttons[4] #buttons[9]
        l2 = buttons[6] #axes[len(axes)-2]
        rl_buttons = [r1,l1,r2,l2]
        l_pad = [axes[0],axes[1],buttons[11]] #buttons[7]
        r_pad = [axes[3],axes[4],buttons[12]] #axes[2] axes[3] buttons[8]
    
    #Planar Motion Assingment
    # Different assignments if cylindrical or cartesian
    if CS_CYLINDRICAL:
        Vtheta = l_pad[1] #Left Joystick Horizontal Axis
        Vr = l_pad[0] #Left Joystick Verical Axis
        
        Vx = Vr*np.cos(arm_theta) - Vtheta*np.sin(arm_theta) #TODO: TESTTTTTTTTTTTTTTTTTT
        Vy = Vr*np.sin(arm_theta) + Vtheta*np.cos(arm_theta)
        Vz = -r_pad[1] #Right Joystick Vertical Axis

    else: # Cartesian
        Vx = -l_pad[0] #Left Joystick Horizontal Axis
        Vy = l_pad[1] #Left Joystick Verical Axis
        Vz = -r_pad[1] #Right Joystick Vertical Axis
    
    #Rotational Motion Assingment 
    Rx = 0
    Ry = 0
    Rz = 0

    #Assign rotation in X to square and circle buttons
    if cir_but: 
        Rx = 1.0
    elif sq_but:
        Rx = -1.0

    #Assign rotation in Y to R1 and L1 buttons
    if r1: 
        Ry = 1.0
    elif l1:
        Ry = -1.0

    #Assign rotation in Z to triangle and x buttons
    if tri_but: 
        Rz = 1.0
    elif x_but:
        Rz = -1.0

    #Might not need anymore
    if l2 == -1.0:
        stateList = [Vx,Vy,Vz,Rx,Ry,Rz,"base"]
    else:
        stateList = [Vx,Vy,Vz,Rx,Ry,Rz,"tool"]

    #if PRINT:
        #print("Right Button Pad",rb_pad)
        #print("Left Button Pad",lb_pad)
        #print("Options",options)
        #print("Share",share)
        #print("Touchpad",touchpad)
        #print("R/L",rl_buttons)
        #print("R Pad",r_pad)
        #print("L Pad",l_pad)
        #print(r2)
        #print(stateList)

    if stateList[-1] == "base":
        x_dot_0 = np.array([[stateList[0]],
                            [stateList[1]],
                            [stateList[2]]])
        omega_dot_0 = np.array([[stateList[3]],
                            [stateList[4]],
                            [stateList[5]]])
        
        baseStateList = np.vstack((x_dot_0, omega_dot_0)).flatten()
        if PRINT:
            print("No transform")

    else:
        R_rol = 0#get roll
        R_pit = 0#get pitch
        R_yaw = 0#get yaw
        roll_matrix = rotation_matrix_roll(R_rol)
        pitch_matrix = rotation_matrix_pitch(R_pit)
        yaw_matrix = rotation_matrix_yaw(R_yaw)

        R_e = np.matmul(yaw_matrix, np.matmul(pitch_matrix, roll_matrix))

        x_dot_0 = np.array([[stateList[0]],
                            [stateList[1]],
                            [stateList[2]]])

        omega_dot_0 = coord_transform(stateList[3],stateList[4],stateList[5],R_e)

        baseStateList = np.vstack((x_dot_0, omega_dot_0)).flatten()

        if PRINT:
            print("Transformation")

    if PRINT:
        print(baseStateList) #baseStateList is the final output

    # Adjust this to control the loop speed
    #pygame.time.wait(100)

    speedButtons = [l_pad[2], r_pad[2]]
    toggleMagnet = options # Use 'options' button for electromagnet on/off
    toggleGripper = share # Use 'share' button for gripper open/close
    sendHome = home
    zPickUp = r2
    # roofPickUp = l2 

    return stateList, speedButtons, toggleGripper, toggleMagnet, sendHome, zPickUp

# Scale the state list to give actual velocities
def scale_state_list(stateList, v_max_planar, v_max_ang):
    #print(stateList)

    # Planar
    stateList[0:3] = [x * v_max_planar for x in stateList[0:3]] #stateList[0::2]*v_max_planar

    # Angular
    stateList[3:6] = [x * v_max_ang for x in stateList[3:6]] #stateList[0::2]*v_max_ang

    #print(stateList)

    return stateList

# Get the scaled inputs from the controller
def get_controller_input_scaled(joystick, v_max_planar, v_max_ang, arm_theta):
    stateList, speedButtons, toggleGripper, toggleMagnet, sendHome, zPickUp = get_controller_input(joystick, arm_theta)

    stateList_scaled = scale_state_list(stateList[0:6], v_max_planar, v_max_ang)

    return stateList_scaled, speedButtons, toggleGripper, toggleMagnet, sendHome, zPickUp



# TODO: 
# pulling l2 to make in 'tool' doesn't work? -> make it pressing '
# 