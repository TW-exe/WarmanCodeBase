//libraries
#include <Arduino.h>
#include <A4988.h>


// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
const float MOTOR_STEPS = 200;

const int WHEELRADIUS = 40; //in mm 

const float wheelCenterRadius = 180; //in mm

//Math constants
const float pi = 3.1415926;

//Pin constants
#define DIR 8
#define STEP 9

#define MS1 10
#define MS2 11
#define MS3 12

#define MS4 5
#define MS5 6
#define MS6 7


A4988 stepperFrontLeft(MOTOR_STEPS, DIR, STEP, MS1, MS2, MS3);   // Stepper motor at the front left
A4988 stepperFrontRight(MOTOR_STEPS, DIR, STEP, MS4, MS5, MS6); //









// Functions 
void move_forward(float distance, float vel) {
  
  int RPM = (vel/(40))*(30/pi); // velocity is in mm per second

  int steps = (100/pi)*(distance/WHEELRADIUS);

  stepperFrontLeft.begin(RPM);
  stepperFrontRight.begin(RPM);

  stepperFrontLeft.enable();
  stepperFrontRight.enable();

  stepperFrontLeft.setMicrostep(1);
  stepperFrontRight.setMicrostep(1);

  stepperFrontLeft.move(steps);
  stepperFrontRight.move(-steps);
}






void move_backward(float distance, float vel){
  
  int RPM = (vel/(40))*(30/pi); // velocity is in mm per second

  int steps = (100/pi)*(distance/WHEELRADIUS);

  stepperFrontLeft.begin(RPM);
  stepperFrontRight.begin(RPM);

  stepperFrontLeft.enable();
  stepperFrontRight.enable();

  stepperFrontLeft.setMicrostep(1);
  stepperFrontRight.setMicrostep(1);

  stepperFrontLeft.move(steps);
  stepperFrontRight.move(-steps);
}

//angVel is in degrees per second
void rotate_clockwise(float degrees, float angVel){
  float vel = (angVel * (2*pi/360)) * wheelCenterRadius;

  float distance = (degrees * (2*pi/360)) * wheelCenterRadius;

  int RPM = (vel/(40))*(30/pi); // velocity is in mm per second

  int steps = (100/pi)*(distance/WHEELRADIUS);

  stepperFrontLeft.begin(RPM);
  stepperFrontRight.begin(RPM);

  stepperFrontLeft.enable();
  stepperFrontRight.enable();

  stepperFrontLeft.setMicrostep(1);
  stepperFrontRight.setMicrostep(1);

  stepperFrontLeft.move(steps);
  stepperFrontRight.move(steps);
}



void rotate_anticlockwise(float degrees, float angVel){
  float vel = (angVel * (2*pi/360)) * wheelCenterRadius;

  float distance = (degrees * (2*pi/360)) * wheelCenterRadius;

  int RPM = (vel/(40))*(30/pi); // velocity is in mm per second

  int steps = (100/pi)*(distance/WHEELRADIUS);

  stepperFrontLeft.begin(RPM);
  stepperFrontRight.begin(RPM);

  stepperFrontLeft.enable();
  stepperFrontRight.enable();

  stepperFrontLeft.setMicrostep(1);
  stepperFrontRight.setMicrostep(1);

  stepperFrontLeft.move(-steps);
  stepperFrontRight.move(-steps);
}







void setup() {
  // put your setup code here, to run once:




}

void loop() {
  // put your main code here, to run repeatedly: