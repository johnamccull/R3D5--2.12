# 2024-Spring-2.12
2024 Spring 2.12 Final Project Team


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

1. change to cylindrical coordinates 
2. add time of flight sensor (since depth is hard to see). This should help with protective stop when there’s too much force 
4. closed loop potentially with time of flight to be able to take the roof off - automate it


Other drivers: Avani, Baran, Kemi

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
----------------



# Extra
5. Define some autonomous behaviors/trajectories for scanning for TIM, picking up and dropping off rubble and picking up and dropping off TIM 
(this includes actuating/deactuating gripper and electromagnet and mapping these to controller/keyboard inputs too)



# Setup Procedure 
Procedure for setting up before starting the competition.

1. Move the robot to home position
2. Mount the gripper
3. Press rst on the PCB
4. Start the serial monitor 
    4a. Put hand in front of IR sensor and make sure gripper serial is working ('I see something hot' should appear)
5. Close the grippers all the way manually
6. Start the python file
7. Open and close grippers to make sure ROM is correct
8. Turn magnet on and off
8. 


# Roles
Amy and Avani's computers work with camera
-> Change timeouts on computers


# UR Arm
## Installation
Install RTDE c++ library: https://sdurobotics.gitlab.io/ur_rtde/installation/installation.html#installation

Install RTDE python client library: https://github.com/UniversalRobots/RTDE_Python_Client_Library/tree/main 


## Useful references
RTDE guide: https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/
