//libraries
#include <Arduino.h>
#include <accelStepper.h>
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <SPI.h>


/////////////////////////////////////////////////////////////////////////////////////////////////////////




// stepper/servo constants
const float MOTOR_STEPS = 200;

const int WHEELRADIUS = 50; //in mm 

const float wheelCenterRadius = 240/2; //in mm

const int maxSpeed = 400; //steps per second

const int accel = 100; // steps per second per second

const int maxServoAngle = 180;

const float armInitialAngle = 90.0;
const float armMotionCoeff = 200;

//Math constants
const float pi = 3.1415926;


//start pins
const int buttonPin = 8; // the pin that the button is attached to
const int ledPin = 13;   // the pin that the LED is attached to (built-in LED on most Arduino boards)

//arm constants
const int armPin1 = 10;
const int armPin2 = 7;


//stepper pins
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




// hand control class
class Hands{

  public:
    Servo servo;
    int openPosition;
    int closedPosition;
    int hitPosition;
    int currentPosition;

    //Constructor 
    Hands(){
      openPosition = 0;
      hitPosition = 20;
      closedPosition = 180;
      currentPosition = openPosition;
    }

    void attach(int pin){
      servo.attach(pin);
      servo.write(currentPosition); // Move servo to initial position
    }

  
    // Method to open the hand
    void open() {
        currentPosition = openPosition;
        servo.write(currentPosition);
        delay(600);

    }

    // Method to hit the ball
    void hit() {
        currentPosition = hitPosition;
        servo.write(currentPosition);
        delay(600);
    }

    // Method to close the hand
    void close() {
        currentPosition = closedPosition;
        servo.write(currentPosition);
        delay(600);
    }

    // Method to close the hand by a certain degree
    void closeByDegrees(int degrees) {
        currentPosition = degrees;
        servo.write(currentPosition);
        delay(600);
    }
  };



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
  stepperFrontRight.moveTo(-steps);
  stepperFrontLeft.moveTo(steps);

  // Synchronize the motors
  while (stepperFrontRight.distanceToGo() != 0 || stepperFrontLeft.distanceToGo() != 0) {
    stepperFrontRight.run();
    stepperFrontLeft.run();
  }
  
  // Reset positions to zero
  stepperFrontLeft.setCurrentPosition(0);
  stepperFrontRight.setCurrentPosition(0);
}


// movement function in mm
void move_backward(float distance){
  int steps = (100/pi)*(distance/WHEELRADIUS);
  stepperFrontRight.moveTo(steps);
  stepperFrontLeft.moveTo(-steps);


  // Synchronize the motors
  while (stepperFrontRight.distanceToGo() != 0 || stepperFrontLeft.distanceToGo() != 0) {
    stepperFrontRight.run();
    stepperFrontLeft.run();
  }
  // Reset positions to zero
  stepperFrontLeft.setCurrentPosition(0);
  stepperFrontRight.setCurrentPosition(0);
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
  
  // Reset positions to zero
  stepperFrontLeft.setCurrentPosition(0);
  stepperFrontRight.setCurrentPosition(0);
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
  
  // Reset positions to zero
  stepperFrontLeft.setCurrentPosition(0);
  stepperFrontRight.setCurrentPosition(0);
}



//arm rotation

void armClockwise(float runTime){
  digitalWrite(armPin2, HIGH);
  delay(runTime);  
  digitalWrite(armPin2,LOW);
}

void armAntiClockwise(float runTime){
  digitalWrite(armPin1, HIGH);
  delay(runTime);  
  digitalWrite(armPin1,LOW);
}



//servo construct 
Hands hand1;
Hands hand2;
//Hands hand3(A2);
//Hands hand4(A3);


///////////////////////////////////////////////////////////////////////////////////////////////


void setup() {
  // put your setup code here, to run once:
  pinMode(buttonPin, INPUT_PULLUP); // initialize the button pin as a pull-up input
  pinMode(ledPin, OUTPUT);          // initialize the LED pin as an output  
  
  pinMode(armPin1,OUTPUT);
  pinMode(armPin2,OUTPUT);

  hand1.attach(A0);

  //servo setup
  hand1.openPosition = 0;
  hand1.closedPosition = 120;
  hand1.hitPosition = 30;

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

  //Deploy arm half way

  //Open hands 1 and 2

  //Deploy arm full way

  //Kick hands in sequential order

  //Retraact arm half way

  //Close hands 1 and 2

  //Navigate to Incinerator

  //Navigate to other Side

  //Open hands 3 and 4

  //Deploy arm full way

  //Kick hands in sequential order

  //Retraact arm half way

  //Close hands 3 and 4

  //Navigate to Incinerator

}
