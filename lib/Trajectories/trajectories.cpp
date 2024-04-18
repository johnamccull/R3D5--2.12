#include "trajectories.h"

JoystickReading joystickReading;
double delta_max = 0.1;
unsigned long setupTime = 3000;

TaskSpace setupLine(TaskSpace initialPosition, TaskSpace nominalPosition, unsigned long time){
    double fraction = static_cast<double>(time) / static_cast<double>(setupTime);
    nominalPosition.x = initialPosition.x + (nominalPosition.x - initialPosition.x) * fraction;
    nominalPosition.y = initialPosition.y + (nominalPosition.y - initialPosition.y) * fraction;
    return nominalPosition;
}

TaskSpace horizontalLine(TaskSpace nominalPosition, double frequency, double amplitude, unsigned long time){
    nominalPosition.x = nominalPosition.x + amplitude*sin(2*M_PI*frequency*time/1000.0);
    return nominalPosition;
}

TaskSpace verticalLine(TaskSpace nominalPosition, double frequency, double amplitude, unsigned long time){
    nominalPosition.y = nominalPosition.y + amplitude*sin(2*M_PI*frequency*time/1000.0);
    return nominalPosition;
}

TaskSpace circle(TaskSpace nominalPosition, double frequency, double radius, unsigned long time){
    nominalPosition.x = nominalPosition.x + radius*cos(2*M_PI*frequency*time/1000.0);
    nominalPosition.y = nominalPosition.y + radius*sin(2*M_PI*frequency*time/1000.0);
    return nominalPosition;
}

TaskSpace spiral(TaskSpace nominalPosition, double frequency, double radius, uint revolutions, unsigned long time){
    nominalPosition.x = nominalPosition.x + radius*cos(2*M_PI*frequency*time/1000.0)*sin(2*M_PI*frequency*time/1000.0);
    nominalPosition.y = nominalPosition.y + radius*sin(2*M_PI*frequency*time/1000.0)*sin(2*M_PI*frequency*time/1000.0);
    return nominalPosition;
}

TaskSpace joystickControl(TaskSpace targetPosition, JoystickReading joystickReading, unsigned long time){
    // ignore steady state error from joystick reading
    if (joystickReading.x > -0.1 && joystickReading.x < 0.1) {
        joystickReading.x = 0.0;
    }
    if (joystickReading.y > -0.1 && joystickReading.y < 0.1) {
        joystickReading.y = 0.0;
    }

    // incremental position control
    TaskSpace newPosition;
    newPosition.x = targetPosition.x + 0.1*joystickReading.x;
    newPosition.y = targetPosition.y + 0.1*joystickReading.y;

    // constrain position to workspace
    newPosition = getClosestPointInWorkspace(newPosition);
    return newPosition;
}

TaskSpace updateSetpoint(TaskSpace initialPosition, TaskSpace nominalPosition, TaskSpace targetPosition, TrajectoryType trajectoryType, unsigned long time){
    if (time < setupTime) {
        return setupLine(initialPosition, nominalPosition, time);
    } else {
        switch (trajectoryType)
        {
        case HORIZONTAL_LINE:
            return horizontalLine(nominalPosition, 0.5, 20, time);
            break;
        case VERTICAL_LINE:
            return verticalLine(nominalPosition, 0.5, 10, time);
            break;
        case CIRCLE:
            return circle(nominalPosition, 0.5, 5, time);
            break;
        case SPIRAL:
            return spiral(nominalPosition, 0.5, 5, 5, time);
            break;
        case JOYSTICK:
            joystickReading = readJoystick();
            return joystickControl(targetPosition, joystickReading, time);
            break;
        default:
            return nominalPosition;
            break;
        }
    }
}

TaskSpace getClosestPointInWorkspace(TaskSpace position) {
    double min_reach = max(0.0, abs(L1 - L2));
    double max_reach = L1 + L2 - delta_max;
    double distance = sqrt(position.x * position.x + position.y * position.y);

    if (distance >= min_reach && distance <= max_reach) {
        // The point is already inside the workspace
        return position;
    } else {
        // Adjust the point to the nearest point on the boundary of the workspace
        TaskSpace closestPoint;
        double scale = distance < min_reach ? min_reach / distance : max_reach / distance;
        closestPoint.x = position.x * scale;
        closestPoint.y = position.y * scale;
        return closestPoint;
    }
}