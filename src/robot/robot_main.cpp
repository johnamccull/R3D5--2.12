#include <Arduino.h>
#include "robot_drive.h"
#include "wireless.h"
#include "util.h"
#include "robot_motion_control.h"
#include "EncoderVelocity.h"

extern RobotMessage robotMessage;
extern ControllerMessage controllerMessage;

int state = 0;
double robotVelocity = 0; // velocity of robot, in m/s
double k = 0; // k is 1/radius from center of rotation circle

extern EncoderVelocity encoders[NUM_MOTORS];
double currPhiL = 0;
double currPhiR = 0;
double prevPhiL = 0;
double prevPhiR = 0;

void setup() {
    Serial.begin(115200);
    setupDrive();
    setupWireless();
}

void loop() {
    // Update velocity setpoints based on trajectory at 50Hz
    EVERY_N_MILLIS(20) {
        if (freshWirelessData) {

           
            double forward = -quadraticMapDouble(controllerMessage.joystick2.x, 1, MAX_FORWARD);
            double turn = -mapDouble(controllerMessage.joystick1.y, -1, 1, -MAX_TURN, MAX_TURN);
            double left = forward - turn;
            double right = forward + turn;

            // tank mode
            //double left = quadraticMapDouble(controllerMessage.joystick1.x, 1, MAX_FORWARD);
            //double right = -quadraticMapDouble(controllerMessage.joystick2.x, 1, MAX_FORWARD);

            updateSetpoints(left, right);
            updateControlEffort();
            
        }
    }  
}