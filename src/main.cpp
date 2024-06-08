//libraries
#include <Arduino.h>
#include <accelStepper.h>




// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
const float MOTOR_STEPS = 200;

const int WHEELRADIUS = 40; //in mm 

const float wheelCenterRadius = 180; //in mm

const int maxSpeed = 1000; //steps per second

const int accel = 50; // steps per second per second

//Math constants
const float pi = 3.1415926;

//Pin constants
#define dirPin1 4
#define pulsePin1 7

#define dirPin2 3
#define pulsePin2 6

const int MS1pin = 8;
const int MS2pin = 9;
const int MS3pin = 10;


AccelStepper stepperFrontRight(1,pulsePin1, dirPin1); 
AccelStepper stepperFrontLeft(1, pulsePin2, dirPin2); 



// Functions 

//micro step initialisation funciton
void microStep(int stepFrac){
  pinMode(MS1pin, OUTPUT);
  pinMode(MS2pin, OUTPUT);
  pinMode(MS3pin, OUTPUT);

  switch (stepFrac)
  {
  case 1:
    digitalWrite(MS1pin, LOW);
    digitalWrite(MS2pin, LOW);
    digitalWrite(MS3pin, LOW);
    break;

  case 2:
    digitalWrite(MS1pin, HIGH);
    digitalWrite(MS2pin, LOW);
    digitalWrite(MS3pin, LOW);
    break;
  
  case 4:
    digitalWrite(MS1pin, LOW);
    digitalWrite(MS2pin, HIGH);
    digitalWrite(MS3pin, LOW);
    break;

  case 8:
    digitalWrite(MS1pin, HIGH);
    digitalWrite(MS2pin, HIGH);
    digitalWrite(MS3pin, LOW);
    break;

  case 16:
    digitalWrite(MS1pin, HIGH);
    digitalWrite(MS2pin, HIGH);
    digitalWrite(MS3pin, HIGH);
    break;
  
  
  default:
    digitalWrite(MS1pin, LOW);
    digitalWrite(MS2pin, LOW);
    digitalWrite(MS3pin, LOW);
    break;
  }
}


// movement function in mm
void move_forward(float distance) {
  int steps = (100/pi)*(distance/WHEELRADIUS);

  stepperFrontRight.setMaxSpeed(maxSpeed);
  stepperFrontRight.setAcceleration(accel);
  stepperFrontRight.moveTo(steps);

  stepperFrontLeft.setMaxSpeed(maxSpeed);
  stepperFrontLeft.setAcceleration(accel);
  stepperFrontLeft.moveTo(-steps);

  stepperFrontRight.run();
  stepperFrontLeft.run();
}



void move_backward(float distance){
  int steps = (100/pi)*(distance/WHEELRADIUS);

  stepperFrontRight.setMaxSpeed(maxSpeed);
  stepperFrontRight.setAcceleration(accel);
  stepperFrontRight.moveTo(steps);

  stepperFrontLeft.setMaxSpeed(maxSpeed);
  stepperFrontLeft.setAcceleration(accel);
  stepperFrontLeft.moveTo(-steps);

  stepperFrontRight.run();
  stepperFrontLeft.run();

}

//angVel is in degrees per second
void rotate_clockwise(float degrees){

  float distance = (degrees * (2*pi/360)) * wheelCenterRadius;

  int steps = (100/pi)*(distance/WHEELRADIUS);

  stepperFrontRight.setMaxSpeed(maxSpeed);
  stepperFrontRight.setAcceleration(accel);
  stepperFrontRight.moveTo(steps);

  stepperFrontLeft.setMaxSpeed(maxSpeed);
  stepperFrontLeft.setAcceleration(accel);
  stepperFrontLeft.moveTo(-steps);

  stepperFrontRight.run();
  stepperFrontLeft.run(); 
}



void rotate_anticlockwise(float degrees){

  float distance = (degrees * (2*pi/360)) * wheelCenterRadius;

  int steps = (100/pi)*(distance/WHEELRADIUS);

  stepperFrontRight.setMaxSpeed(maxSpeed);
  stepperFrontRight.setAcceleration(accel);
  stepperFrontRight.moveTo(steps);

  stepperFrontLeft.setMaxSpeed(maxSpeed);
  stepperFrontLeft.setAcceleration(accel);
  stepperFrontLeft.moveTo(-steps);

  stepperFrontRight.run();
  stepperFrontLeft.run();
}


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}