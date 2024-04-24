#include "Robot.h"

Robot::Robot() : leftMotor_(1, 2, 3), rightMotor_(4, 5, 6) {
    // Constructor code
}

void Robot::moveForward() {
    // Code to move the robot forward
}

void Robot::moveBackward() {
    // Code to move the robot backward
}

long Robot::getLeftMotorPosition() {
    return leftMotor_.getPosition();
}

long Robot::getRightMotorPosition() {
    return rightMotor_.getPosition();
}
