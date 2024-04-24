#include <Arduino.h>
#include "Robot.h"

Robot myRobot;

void setup() {
    // Setup code
}

void loop() {
    myRobot.moveForward();
    delay(1000);
    myRobot.moveBackward();
    delay(1000);
}
