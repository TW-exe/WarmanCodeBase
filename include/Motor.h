#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
public:
    Motor(int pin1, int pin2, int encoderPin);
    void setSpeed(int speed);
    long getPosition();
private:
    int pin1_;
    int pin2_;
    int encoderPin_;
};

#endif