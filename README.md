# 2024-Spring-2.12
Team R3D5 Lab Section R3-5 <br>
**Members**: Amir White, 
Amy Mohamed,
Avani Narula,
Baran Mensah,
Elysia Yuan,
Harvey Merton,
John McCullough,
Kemi Chung,
Malachi Macon,
Ottavia Personeni,

# CODE STRUCTURE
## UR5
- UR_3_PS4_Control.py (main branch)
---> This sends commands from the PS4 controller to the ESP in order to manipulate our gripper and electromagnet;
  Accessed by M
- UR_3_Main_Arm_Control.py (main branch)
---> Run on the computer during the competition and runs entire loop, reading both keyboard and controller input
- UR_3_gripper_control.cpp (main branch)
---> Run on ESP and translates controller commands into movements to control servos and magnet

## Mobile Robot
(please checkout to mobile-robot branch and read respective README)

# Operators
-Mobile Robot Driver: John McCullough  
-Skycam Operator(s): Malachi Macon, Baran Mensah  
-UR5 Operator: Harvey Merton  
-Presenters: Amir White, Avani Narula, Baran Mensah, Elysia Yuan, Ottavia Personeni

# Competition Procedure
## Supplies
1. Data transfer cords 
2. M4 screws and hex key
3. UR5 screws and hex key
6. Mini screwdriver (for power terminals)
7. Wire strippers 
8. Power source (for magnet)
9. Battery & its charger 
10. Many zipties (will have to daisy chain the ones we have from lab)
11. Mount 
12. Gripper 
13. Tape & velcro 
14. Controller 
15. Ethernet adapter 
16. Computer chargers 
17. Adapters
18. Jumper wires
19. Good vibes
## Setup
1. Move the robot to home position
2. Mount the gripper
3. Press rst on the PCB
4. Start the serial monitor 
    4a. Put hand in front of IR sensor and make sure gripper serial is working ('I see something hot' should appear)
5. Close the grippers all the way manually
6. Start the python file
7. Open and close grippers to make sure ROM is correct
8. Turn magnet on and off
## During
1. From Home Position, press 3 which will go to house 3 first 
2. detect IR 
    2a. if no IR:
        3a. go to house 2 
        3b. detect IR 
    4a. if no IR:
        5a. go to house 1
3. get someone to tell the driver of the orienation of the house and which letter to press 
4. always grab the taller roof first 
5. grab the shorter roof 
6. go down over TIM with toggle R2
7. adjust accordingly 
8. pick up Tim with claw and toggle R2 again
9. drop TIM over mobile robot

----------------

# TODO
~~Electromagnet + test on roof- Amir~~  
~~IR integration + testing - Amy~~  
~~Singularity - Avani + Baran~~  
~~Test with additional webcam -~~  
~~Command to specific z height + print out z height - Kemi (Display TIM height)~~  
~~Make controls smoother ~~

Automatic Rubble dropoff
Speed control for precise movement (mobile and UR5)
Practice runs !!

~~2. add time of flight sensor (since depth is hard to see). This should help with protective stop when there’s too much force~~
4. closed loop potentially with time of flight to be able to take the roof off - automate it


# Done
1. prevent going into singularity, OR: if robot is in singularity, get out!!!!!
2. Integrate with IR sensor
3. Have button that goes down to particular height to pick tim up, then up to a particular height to allow movement -> Can move down, but not up yet
4. Keep robot hand parallel with ground (this should be easy - just start it parallel to the ground!)
---> Print-outs make it difficult to see useful information
---> Claw opens too far and knocks ESP-32 out
---> claw opened a bit and dropped TIM
1. USB-C cable is too short
3. Magnet doesn't engage unless perfectly parallel (don't change orientation after start pose)
4. Difficult to see depth with camera - perhaps move backwards or have some form of depth sensor (use reflection to judge depth)
5. test cylindrical coordinates
6. Define some autonomous behaviors/trajectories for scanning for TIM and roof removal
----------------

# UR Arm Notes
## Installation
Install RTDE c++ library: https://sdurobotics.gitlab.io/ur_rtde/installation/installation.html#installation

Install RTDE python client library: https://github.com/UniversalRobots/RTDE_Python_Client_Library/tree/main 


## Useful references
RTDE guide: https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/
