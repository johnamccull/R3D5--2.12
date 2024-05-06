#include <Arduino.h>
#include "robot_drive.h"
#include "wireless.h"
#include "util.h"
#include "robot_motion_control.h"
#include "EncoderVelocity.h"

extern RobotMessage robotMessage;
extern ControllerMessage controllerMessage;

void setup() {
    Serial.begin(115200);
    setupDrive();
    setupWireless();
}

void loop() {
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