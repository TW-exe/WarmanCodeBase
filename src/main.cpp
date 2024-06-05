//libraries
#include <Arduino.h>
#include <A4988.h>


// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200

#define WHEELRADIUS 40//in mm 

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
  int RPM = vel/(40); // velocity is in mm per second
  stepperFrontLeft.begin(RPM);
  stepperFrontRight.begin(RPM);

  stepperFrontLeft.enable();
  stepperFrontRight.enable();

  stepperFrontLeft.setMicrostep(1);
  stepperFrontRight.setMicrostep(1);

  stepperFrontLeft.move();
  stepperFrontRight.move();


}


void move_backward(){

}


void rotate_clockwise(){

}


void rotate_anticlockwise(){

}






void setup() {
  // put your setup code here, to run once:




}

void loop() {
  // put your main code here, to run repeatedly: