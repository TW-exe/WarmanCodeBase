//libraries
#include <Arduino.h>
#include <AccelStepper.h>




// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
const float MOTOR_STEPS = 200;

const int WHEELRADIUS = 40; //in mm 

const float wheelCenterRadius = 180; //in mm

//Math constants
const float pi = 3.1415926;

//Pin constants
#define dirPin1 8
#define pulsePin1 9

#define dirPin2 8
#define pulsePin2 9



AccelStepper stepperFrontRight(1,pulsePin1, dirPin1); 
AccelStepper stepperFrontLeft(1, pulsePin2, dirPin2); 



/*



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

*/


/*


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

*/

/*


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

*/


int speed1;
int speed2;



void setup() {
  // put your setup code here, to run once:
  stepperFrontRight.setMaxSpeed(200);
  stepperFrontRight.setAcceleration(100);
  stepperFrontRight.moveTo(200);

  stepperFrontLeft.setMaxSpeed(200);
  stepperFrontLeft.setAcceleration(100);
  stepperFrontLeft.moveTo(-200);



}

void loop() {
  // put your main code here, to run repeatedly:
  stepperFrontRight.run();
  stepperFrontLeft.run();
}