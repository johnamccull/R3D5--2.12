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
        //followTrajectory();
        if (freshWirelessData) {

            //double forward = -mapDouble(controllerMessage.joystick1.x, -1, 1, -MAX_FORWARD, MAX_FORWARD);
            double forward = -quadraticMapDouble(controllerMessage.joystick1.x, 1, MAX_FORWARD);
            double turn = mapDouble(controllerMessage.joystick2.y, -1, 1, -MAX_TURN, MAX_TURN);

            double left = forward - turn;
            double right = forward + turn;
            
            updateSetpoints(left, right);
            updateControlEffort();
        }
    }

    // Update PID at 200Hz
    /**
    EVERY_N_MILLIS(5) {
        updateControlEffort();
    }
    **/

    // Send and print robot values at 200Hz
    /**
    EVERY_N_MILLIS(5) {
        //updateOdometry();
        // take angles from traction wheels only since they don't slip
        currPhiL = encoders[2].getPosition();
        currPhiR = -encoders[3].getPosition();
        
        double dPhiL = currPhiL - prevPhiL;
        double dPhiR = currPhiR - prevPhiR;
        prevPhiL = currPhiL;
        prevPhiR = currPhiR;

        float dtheta = r/(2*b)*(dPhiR-dPhiL);
        float dx = r/2.0 * (cos(robotMessage.theta)*dPhiR + cos(robotMessage.theta)*dPhiL);
        float dy = r/2.0 * (sin(robotMessage.theta)*dPhiR + sin(robotMessage.theta)*dPhiL);

        // Update robot message 
        robotMessage.millis = millis();
        robotMessage.x += dx;
        robotMessage.y += dy;
        robotMessage.theta += dtheta;
        
        sendRobotData();

        Serial.printf("x: %.2f, y: %.2f, theta: %.2f\n",
                    robotMessage.x, robotMessage.y, robotMessage.theta);
    }
    **/
  
}