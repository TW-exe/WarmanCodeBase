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




//flap construct
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



//arm rotations
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



//Functions for Flaps
void right_flap_closed(){
  servo3.write(20);
  delay(600);
}


void right_flap_open(){
  servo3.write(180);
  delay(600);
}


void left_flap_closed(){
  servo4.write(180);
  delay(600);
}


void left_flap_open(){
  servo4.write(20);
  delay(600);
}


void shake(){
  //shake code
  for(int i=0;i<6;i++){
  move_forward(100);
  move_backward(100);
  }
}



//hand construct
Hands hand1;
Hands hand2;

///////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(buttonPin, INPUT_PULLUP); // initialize the button pin as a pull-up input
  pinMode(ledPin, OUTPUT);          // initialize the LED pin as an output  
  
  pinMode(armPin1,OUTPUT);
  pinMode(armPin2,OUTPUT);

  hand1.attach(A0);
  hand2.attach(A1);
  
  servo4.attach(A2);
  servo3.attach(A3);
  

  hand1.openPosition = 5;
  hand1.closedPosition = 90;
  hand1.hitPosition = 17;

  hand2.openPosition = 0;
  hand2.closedPosition = 90;
  hand2.hitPosition = 20;

  //stepper setup
  microStep(1);
  stepperFrontRight.setMaxSpeed(maxSpeed);
  stepperFrontRight.setAcceleration(accel);
  stepperFrontLeft.setMaxSpeed(maxSpeed);
  stepperFrontLeft.setAcceleration(accel);

  right_flap_closed();
  delay(5000);
  left_flap_closed();
  
    
}
///////////////////////////////////////////////////////////////////////////////////////////////



void loop() {
  //whatever you want to test put it under the push start function
  pushStart();
  


  /*
  //Deploy arm half way
  armClockwise(800);

  //Open hands 1 and 2
  hand1.open();
  delay(1000);

  //Deploy arm full way
  armClockwise(400);
  //Kick hands in sequential order
  left_flap_open();
  hand1.hit();
  delay(1000);
  //Retraact arm half way
  armAntiClockwise(700);
  //Close hands 1 and 2
  hand1.close();
  hand2.close();

  //Navigate to Incinerator
  move_backward(200);
  delay(400);
  rotate_anticlockwise(90);
  move_forward(300);

  //hips dont lie
  shake();

  //Navigate to other Side

  //Open hands 3 and 4

  //Deploy arm full way

  //Kick hands in sequential order

  //Retraact arm half way

  //Close hands 3 and 4

  //Navigate to Incinerator
  */
}
