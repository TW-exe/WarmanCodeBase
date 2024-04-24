#ifndef ROBOT_H
#define ROBOT_H

#include "Motor.h"

class Robot {
public:
    Robot();
    void moveForward();
    void moveBackward();
    long getLeftMotorPosition();
    long getRightMotorPosition();
private:
    Motor leftMotor_;
    Motor rightMotor_;
};

#endif
