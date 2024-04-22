#include <Arduino.h>
#include "util.h"
#include "robot_drive.h"
#include "EncoderVelocity.h"
#include "wireless.h"
#include "robot_motion_control.h"

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

// Sets the desired wheel velocities based on desired robot velocity in m/s
// and k curvature in 1/m representing 1/(radius of curvature)
void setWheelVelocities(float robotVelocity, float k){
    double left = (robotVelocity - k*b*robotVelocity)/r;
    double right = 2*robotVelocity/r  - left;
    updateSetpoints(left, right);
}

// Makes robot follow a trajectory
void followTrajectory() {

    if (freshWirelessData) {
        double forward = abs(controllerMessage.joystick1.y) < 0.1 ? 0 : mapDouble(controllerMessage.joystick1.y, -1, 1, -MAX_FORWARD, MAX_FORWARD);
        double turn = abs(controllerMessage.joystick1.x) < 0.1 ? 0 : mapDouble(controllerMessage.joystick1.x, -1, 1, -MAX_TURN, MAX_TURN);
        updateSetpoints(forward - turn, forward + turn);
    }

}

void updateOdometry() {
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
}

