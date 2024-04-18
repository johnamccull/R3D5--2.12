#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#include "kinematics.h"
#include "util.h"
#include "joystick.h"

enum TrajectoryType {
    HORIZONTAL_LINE,
    VERTICAL_LINE,
    CIRCLE,
    JOYSTICK,
    SPIRAL,
    RECTANGLE
};

TaskSpace setupLine(TaskSpace initialPosition, TaskSpace nominalPosition, unsigned long time);
TaskSpace horizontalLine(TaskSpace nominalPosition, double frequency, double amplitude, unsigned long time);
TaskSpace verticalLine(TaskSpace nominalPosition, double frequency, double amplitude, unsigned long time);
TaskSpace circle(TaskSpace nominalPosition, double frequency, double radius, unsigned long time);
TaskSpace spiral(TaskSpace nominalPosition, double frequency, double radius, uint revolutions, unsigned long time);
TaskSpace joystickControl(TaskSpace actualPosition, JoystickReading joystickReading, unsigned long time);
TaskSpace updateSetpoint(TaskSpace initialPosition, TaskSpace nominalPosition, TaskSpace actualPosition, TrajectoryType trajectoryType, unsigned long time);
TaskSpace getClosestPointInWorkspace(TaskSpace position);

extern JoystickReading joystickReading;
#endif