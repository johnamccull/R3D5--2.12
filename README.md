# 2024-Spring-2.12 Team 3 Mobile Robot
2024 Spring 2.12 Final Project Team 3 (Thursday 3-5pm)

# Mobile Robot

## Pinouts

Define controller joystick1 and joystick2 X and Y pins in header file: controller_pinout.h

Joystick1: turning

Joystick2: forward/backward 

Define robot motor pins in header file: robot_pinout.h

## MAC Addresses

Get MAC addresses of robot/controller: upload get_mac.cpp to the particular microcontroller and read printout of its MAC address

Define MAC addresses of robot/controller as constants robotAddr and controllerAddr, respectively, in the header file: wireless.h

## Running the robot

Set maximum speeds for forward/backward drive and turning by defining MAX_FORWARD and MAX_TURN, respectively, in the header file: robot_drive.h

Upload the following files to the robot: 

robot_drive.cpp - defines robot driving function given speed commands

robot_main.cpp - main program execution loop on robot

robot_wireless.cpp - defines wireless communication functions used by robot to receive data from controller

## Running the controller

Upload the following files to the controller: 

controller_main.cpp - main program execution loop on controller

controller_wireless.cpp - defines wireless communication functions used by controller to send data to robot for commanding motor speeds