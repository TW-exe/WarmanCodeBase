//libraries
#include <Arduino.h>
#include <accelStepper.h>
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <SPI.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
const float MOTOR_STEPS = 200;

const int WHEELRADIUS = 50; //in mm 

const float wheelCenterRadius = 380/2; //in mm

const int maxSpeed = 400; //steps per second

const int accel = 100; // steps per second per second

const int maxServoAngle = 180;

//Math constants
const float pi = 3.1415926;


//start constants
const int buttonPin = 8; // the pin that the button is attached to
const int ledPin = 13;   // the pin that the LED is attached to (built-in LED on most Arduino boards)


//stepper constants
#define pulsePin1 3
#define dirPin1 2

#define pulsePin2 5
#define dirPin2 4


const int MS1pin = 13;
const int MS2pin = 12;
const int MS3pin = 11;

//Stepper construct
AccelStepper stepperFrontRight(1,pulsePin1, dirPin1); 
AccelStepper stepperFrontLeft(1, pulsePin2, dirPin2); 

//servo construct 
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;




///////////////////////////////////////////////////////////////////////////////////////////////


//Start Function
void pushStart(){
  while(true){
    int buttonState = digitalRead(buttonPin);
    if (buttonState == LOW) { // button is pressed
      digitalWrite(ledPin, HIGH); // turn the LED on
      delay(1000);
      digitalWrite(ledPin, LOW); // turn the LED on
      delay(400);

      break;

    } 

  }
}




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
  stepperFrontRight.moveTo(steps);
  stepperFrontLeft.moveTo(-steps);

  // Synchronize the motors
  while (stepperFrontRight.distanceToGo() != 0 || stepperFrontLeft.distanceToGo() != 0) {
    stepperFrontRight.run();
    stepperFrontLeft.run();
  }
}


// movement function in mm
void move_backward(float distance){
  int steps = (100/pi)*(distance/WHEELRADIUS);
  stepperFrontRight.moveTo(-steps);
  stepperFrontLeft.moveTo(steps);

  // Synchronize the motors
  while (stepperFrontRight.distanceToGo() != 0 || stepperFrontLeft.distanceToGo() != 0) {
    stepperFrontRight.run();
    stepperFrontLeft.run();
  }
}

//rotation of system in degrees
void rotate_clockwise(float degrees){
  float distance = (degrees * (2*pi/360)) * wheelCenterRadius;
  int steps = (100/pi)*(distance/WHEELRADIUS);
  stepperFrontRight.moveTo(steps);
  stepperFrontLeft.moveTo(steps);

  // Synchronize the motors
  while (stepperFrontRight.distanceToGo() != 0 || stepperFrontLeft.distanceToGo() != 0) {
    stepperFrontRight.run();
    stepperFrontLeft.run();
  }
}



//rotation of system in degrees
void rotate_anticlockwise(float degrees){
  float distance = (degrees * (2*pi/360)) * wheelCenterRadius;
  int steps = (100/pi)*(distance/WHEELRADIUS);
  stepperFrontRight.moveTo(-steps);
  stepperFrontLeft.moveTo(-steps);

  // Synchronize the motors
  while (stepperFrontRight.distanceToGo() != 0 || stepperFrontLeft.distanceToGo() != 0) {
    stepperFrontRight.run();
    stepperFrontLeft.run();
  }
}




// Servo functions 


void servoStartUp(){

}





void servo1Mov(int setAngle, bool strike){
  int angle = 180-setAngle;
  servo1.write(angle);
  delay(600);

  if(strike){
    servo1.write(angle-20);
    delay(600);
    
    servo1.write(angle);
    delay(600);
  }
}

void servo2Mov(int setAngle, bool strike){
  int angle = 180-setAngle;
  servo2.write(angle);
  delay(600);

  if(strike){
    servo2.write(angle-20);
    delay(600);
    
    servo2.write(angle);
    delay(600);
  }
}

void servo3Mov(int setAngle, bool strike){
  int angle = setAngle;
  servo3.write(angle);
  delay(600);

  if(strike){
    servo3.write(angle+20);
    delay(600);
    
    servo3.write(angle);
    delay(600);
  }
}

void servo4Mov(int setAngle, bool strike){
  int angle = setAngle;
  servo4.write(angle);
  delay(600);

  if(strike){
    servo4.write(angle+20);
    delay(600);
    
    servo4.write(angle);
    delay(600);
  }
}





///////////////////////////////////////////////////////////////////////////////////////////////





void setup() {
  // put your setup code here, to run once:
  pinMode(buttonPin, INPUT_PULLUP); // initialize the button pin as a pull-up input
  pinMode(ledPin, OUTPUT);          // initialize the LED pin as an output  

  //servo setup
  servo1.attach(A0);
  servo2.attach(A1);
  servo3.attach(A2);
  servo4.attach(A3);



  //stepper setup
  microStep(1);
  stepperFrontRight.setMaxSpeed(maxSpeed);
  stepperFrontRight.setAcceleration(accel);
  stepperFrontLeft.setMaxSpeed(maxSpeed);
  stepperFrontLeft.setAcceleration(accel);
    

}



///////////////////////////////////////////////////////////////////////////////////////////////



void loop() {
  //whatever you want to test put it under the push start function
  pushStart();



}
