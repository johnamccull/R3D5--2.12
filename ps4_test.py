import pygame
import numpy as np

#TODO
#Always  put rotations in 
#Spherical Coordinates
#Change layout slightly
#Tool pose speed/angular speed increment/decrement [Not necesaary]


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

# Main loop
running = True
while running:
    # Check for events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Get joystick axes and buttons
    axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
    axes = smooth_data(axes)
    buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]

    # Print axes and buttons
    #print("Axes:", axes)
    #print("Buttons:", buttons)

    #Group to PS4 Layout

    # Right Button Pad
    tri_but = buttons[3]
    sq_but = buttons[2]
    cir_but = buttons[1]
    x_but = buttons[0]
    # up_but = buttons[11]
    # l_but = buttons[13]
    # r_but = buttons[14]
    # d_but = buttons[12]
    rb_pad = [tri_but,sq_but,cir_but,x_but]
    # lb_pad = [up_but,l_but,r_but,d_but]
    options = buttons[6]
    share = buttons[4]
    touchpad = buttons[len(buttons)-1]
    r1 = buttons[10]
    r2 = axes[len(axes)-1]
    l1 = buttons[9]
    l2 = axes[len(axes)-2]
    rl_buttons = [r1,l1,r2,l2]
    l_pad = [axes[0],axes[1],buttons[7]]
    r_pad = [axes[2],axes[3],buttons[8]]
    
    #Planar Motion Assingment
    Vx = l_pad[0] #Left Joystick Horizontal Axis
    Vy = -l_pad[1] #Left Joystick Verical Axis
    Vz = r_pad[0] #Right Joystick Vertical Axis
    
    #Rotational Motion Assingment 
    Rx = 0
    Ry = 0
    Rz = 0

    #Assign rotation in X to square and circle buttons
    if cir_but: 
        Ry = 1.0
    elif sq_but:
        Ry = -1.0

    #Assign rotation in Y to R1 and L1 buttons
    if r1: 
        Ry = 1.0
    elif l1:
        Ry = -1.0

    #Assign rotation in Z to triangle and x buttons
    if tri_but: 
        Ry = 1.0
    elif x_but:
        Ry = -1.0

    #
    if l2 == -1.0:
        stateList = [Vx,Vy,Vz,Rx,Ry,Rz,"base"]
    else:
        stateList = [Vx,Vy,Vz,Rx,Ry,Rz,"tool"]

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
        baseStateList = stateList[:-1]
        #print("No transform")
        baseStateVector = np.array(baseStateList)
    else:
        R_rol = 0#get roll
        R_pit = 0#get pitch
        R_yaw = 0#get yaw
        roll_matrix = rotation_matrix_roll(R_rol)
        pitch_matrix = rotation_matrix_pitch(R_pit)
        yaw_matrix = rotation_matrix_yaw(R_yaw)

        R_e = np.matmul(yaw_matrix, np.matmul(pitch_matrix, roll_matrix))

        x_dot_e = np.array([[stateList[0]],
                            [stateList[1]],
                            [stateList[2]]])

        omega_dot_e = np.array([[stateList[3]],
                               [stateList[4]],
                               [stateList[5]]])
        
        x_dot_0 = np.matmul(R_e,x_dot_e)
        omega_dot_0 = np.matmul(R_e,omega_dot_e)

        baseStateList = np.vstack((x_dot_0, omega_dot_0)).flatten()
        #print("Transformation")

    print(baseStateList) #baseStateList is the final output

    # Adjust this to control the loop speed
    pygame.time.wait(100)

