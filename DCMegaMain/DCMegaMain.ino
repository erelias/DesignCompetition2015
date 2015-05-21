/******************************************************************
Design Competition 2015 Code for the Arduino Mega 2560
Eric Elias, Paul Green, Jane Miller, and Christopher Pontisakos
******************************************************************/


//RGB LED fil
#include <Adafruit_NeoPixel.h>
#include <avr/power.h>

//UltraSonic Library
//Sorry Chris
#include <NewPing.h>

//Timer stuff
#include <Event.h>
#include <Timer.h>



enum stateMachine{
  prePlanned,
  search,  
  found,
  killSwitch,
};
stateMachine robotState;





/************************
Ultrasonic Sensor
************************/

/***********
Front Sensor
***********/
//Pins and stuff
#define TRIGGER_FRONT  52  // Because I can
#define ECHO_FRONT     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DIST_FRONT 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define WALL_LIMIT 30
NewPing fSonar(TRIGGER_FRONT, ECHO_FRONT, MAX_DIST_FRONT);

//Moving average
boolean fSonarArray[5];
int fSonarPos;
int fSonarSum;



/************************
Front Retroreflective Sensors
************************/
#define RETRO_THRESH            400
const int leftRetroPin = 15;

/*************************
Color Sensor 
*************************/
//analog pin for color sensor
const int frontColorPin = 5;
//the threshold between purple and white
const int colorThreshold = 500;

//moving average
const int ColorSize = 5;
int frontColorAvg[colorSize];
int frontColorPos;
int frontColorSum;
int frontColorVal;

int backColorAvg[colorSize];
int backColorPos;
int backColorSum;
int backColorVal;

//tells us what color we are on
boolean isOnWhite;
enum color{
  purple,
  white
};
color startColor;
color currColor;
color oldColor;

/*************************
Break Beam - not going to be an interrupt
*************************/
//Laser/phototransistor in front of the roller
//If it is low (broken), something is being pushed
//I am attempting to program an interrupt that acts based on  
//when there is a change in the state of the pin
//See: http://www.arduino.cc/en/Reference/AttachInterrupt

//interrupt 0 is on digital pin 2
//const int breakPin = 0;

//When the motor is running, the pin randomly goes high, triggering the interrupt.
//in order to change this, we want 5 in a row to be the same before the 
//interrupt occurs
//int breakArray[5];
/*A variable should be declared volatile whenever its value can be changed by something beyond the control of 
the code section in which it appears, such as a concurrently executing thread. 
In the Arduino, the only place that this is likely to occur is in sections of code associated with interrupts. */

//volatile int state = LOW;

/*************************
HBridge Controls
*************************/
//Left 
// B
// Enable: 3
const int enableL = 3;
// Phase: 7
const int phaseL = 7;

//Right
// A
// Enable: 5
const int enableR = 5;
// Phase: 4
const int phaseR = 4;

int hbridgepins[] = {enableL,phaseL,enableR,phaseR};

enum driveDirection{
  forward,
  left,
  right,
  robotstop,
  backwards
};

driveDirection steering;

int leftSpeed;
int leftCounter;
int rightSpeed;
int rightCounter;

/********************************
Encoder Stuff
********************************/
int leftEncoderPin = 21; //interrupt 2
int rightEncoderPin = 20; //interrupt 3
volatile int leftEncoder = 0;
volatile int rightEncoder = 0;
Timer encoderTimer;

/*******************************
Timer
*******************************/
Timer t; //Sets up a timer to shut off the motors after 3 minutes

/******************************
Test Stuff
******************************/
int numColorChange;
int fullBackTime;
int forwardOneTime;
int halfBackTime;
int leftNinetyTime;
int rightNinetyTime;
void setup(){
  
  numColorChange = 0;
  fullBackTime = 1525;
  forwardOneTime = 1500;
  halfBackTime = 400;
  leftNinetyTime = 360;
  rightNinetyTime = 360;
  //RGB LED (may not be needed)
 
  
  //initializes the HBridge pins
  int i = 0;
  for (i=0; i <= 3; i++){
    pinMode(hbridgepins[i], OUTPUT);
  }
  
  //Color sensor moving average + ultrasonic moving average
  frontColorSum = 0;
  frontColorPos = 0;
  backColorSum = 0;
  backColorPos = 0;
  fSonarPos = 0;
  fSonarSum = 0;
  for (i = 0; i < 10; i++){
    isOnWhite = frontColorCheck();
    backColorCheck();
    frontSonarCheck();
  }
  //Tells us what color we started on
  //if we start on white, we want to go to purple
  if(isOnWhite){
     Serial.println("We started on a White Square");
     startColor = white;
     
  }
  else{
    Serial.println("We started on a Purple Square");
    startColor = purple;
    
  }

 for(i = 0; i < 5; i++){
    //breakArray[i] = state; 
    Serial.println(fSonar.ping_cm());
    
  }
  
 
  
  //causes the robot to stop moving after 3 minutes (1000 millisec/sec * 60 sec/min * 3 min)
  t.every((long)1000*60*3,kill);

  leftSpeed = 200;//240;
  leftCounter = 0;
  rightSpeed = 210;//255;
  rightCounter = 0;

 //  goForward();
  robotState = prePlanned;
  
  Serial.begin(9600);  
  
  
}

void loop() {
 /*
 t.update();
 switch (robotState){
    case prePlanned:
      Serial.println("prePlanned");
      if(frontSonarCheck()){
         goBackwards();
         delay(100);
         Serial.println("stop!");
         
         robotState = search; 
        
      }
      break;
    case search:
      //if(startColor = purple){
         goLeft(); 
      //}
      if(analogRead(leftRetroPin) > RETRO_THRESH){
        robotState = found;
      }
      else{
        goLeft();
      }
      if(steering != robotstop){
        robot_stop();
      }
      break;
    case found:
      //if(robotState !=robotstop){
         robot_stop(); 
      //566.}
      break;
     case killSwitch:
       robot_stop();
       
  }*/
  
switch (robotState){
  case prePlanned:
   /*goForward();
   frontColorCheck();
   if(currColor != oldColor){
     numColorChange++;
     if(numColorChange == 3){
       
       halfBack();
       forwardOneSquare();
       rightNinety();
       forwardOneSquare();
       rightNinety();
       numColorChange = 0;*/
       forwardOneSquare();
       robotState = killSwitch;
       
  //   }
    // }
     break;
   case killSwitch:
     robot_stop();
     break;
}

  
}

//Pre planned path code
//!!!!!!!!!!!!!!!!!!
//Battery Dependent
//!!!!!!!!!!!!!!!!!!

void leftNinety(){
 goLeft();
 delay(leftNinetyTime);
 robot_stop();
 delay(1000); 
}

void rightNinety(){
  goRight();
 delay(rightNinetyTime);
 robot_stop();
 delay(1000);
  
}

void forwardOneSquare(){
 goForward();
 delay(forwardOneTime);
 robot_stop();
 delay(1000); 
  
}

void halfBack(){
  goBackwards();
  delay(halfBackTime);
  robot_stop();
  delay(1000);
}

void fullBack(){
  goBackwards();
  delay(fullBackTime);
  robot_stop();
  delay(1000);
}


//Motor Controls
void setL(int phase, int enable){
  digitalWrite(phaseL, phase);
  analogWrite(enableL,enable);
}

void setR(int phase, int enable){
  digitalWrite(phaseR, phase);
  analogWrite(enableR, enable);
}

//Direction Controls
void goForward(){
  setL(LOW,leftSpeed);
  setR(LOW,rightSpeed);
  steering = forward;
}

void goLeft(){
  setL(HIGH,0);
  setR(LOW,255);
  steering = left;
  
}

void goRight(){
  setL(LOW,255);
  setR(HIGH,0);
  steering = right;
}

void robot_stop(){
  setL(LOW,0);
  setR(LOW,0);
  steering = robotstop;
  Serial.println("i stopped");
}

void goBackwards(){
 setL(HIGH,255-leftSpeed);
 setR(HIGH,255-rightSpeed); 
 steering = backwards;
}


boolean frontColorCheck(){
  //Checks to see what color the robot is on
  //returns 1 if on white, 0 if is on purple
  frontColorSum -= frontColorAvg[frontColorPos];
  frontColorAvg[frontColorPos] = analogRead(frontColorPin);
  frontColorSum += frontColorAvg[frontColorPos];
  frontColorPos += 1;
  if (frontColorPos >= colorSize){
    frontColorPos = 0;  
  }
  oldColor = currColor;
  frontColorVal = frontColorSum/colorSize;
  
  if (frontColorVal <= colorThreshold){
    currColor = purple;
    return 0;
    
  }
  currColor = white;
  
  return 1;
  
}

boolean backColorCheck(){
  //Checks to see what color the robot is on
  //returns 1 if on white, 0 if is on purple
  backColorSum -= backColorAvg[backColorPos];
  backColorAvg[backColorPos] = analogRead(backColorPin);
  backColorSum += backColorAvg[backColorPos];
  backColorPos += 1;
  if (backColorPos >= colorSize){
    backColorPos = 0;  
  }
  //oldColor = currColor;
  backColorVal = backColorSum/colorSize;
  
  if (backColorVal <= colorThreshold){
    //currColor = purple;
    return 0;
    
  }
  currColor = white;
  
  return 1;
  
}

//Uses the sonar to check if there's a wall in front of it
//returns true if there is an object within a certain distance 
boolean frontSonarCheck(){
  fSonarSum -= fSonarArray[fSonarPos];
  int uS = fSonar.ping_cm();
  Serial.println(uS);
  if(uS == 0){
    uS = 200;
  }  
  fSonarArray[fSonarPos] = 0;
  if(uS<= WALL_LIMIT){
     fSonarArray[fSonarPos] = 1; 
  }
  fSonarSum += fSonarArray[fSonarPos];
  fSonarPos += 1;
  if (fSonarPos >= 5){
    fSonarPos = 0;  
  }
  if(fSonarSum >= 4){
   return true; 
  }
  
  
  return false;
}

void encoderReset(){
 leftEncoder = 0;
 rightEncoder = 0; 
  
}


void kill(){
 robotState = killSwitch;
  
}
