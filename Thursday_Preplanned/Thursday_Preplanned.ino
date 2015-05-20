/******************************************************************
Design Competition 2015 Code for the Arduino Mega 2560
Eric Elias, Paul Green, Jane Miller, and Christopher Pontisakos
******************************************************************/
#include <avr/power.h>
//Timer stuff
#include <Event.h>
#include <Timer.h>

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

Timer t; //Sets up a timer to shut off the motors after 3 minutes


void setup(){

  //initializes the HBridge pins
  int i = 0;
  for (i=0; i <= 3; i++){
    pinMode(hbridgepins[i], OUTPUT);
  }
 
  
  Serial.begin(9600);  
  
  //causes the robot to stop moving after 3 minutes (1000 millisec/sec * 60 sec/min * 3 min)
  t.every((long)1000*60*3,kill);
}

void loop() {
  t.update();
 
  
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
  setL(LOW,230);
  setR(LOW,255);
  steering = forward;
}

void goLeft(){
  setL(LOW,0);
  setR(LOW,255);
  steering = left;
  Serial.println("im driving left");
}

void goRight(){
  setL(LOW,255);
  setR(LOW,0);
  steering = right;
}

void robot_stop(){
  setL(LOW,0);
  setR(LOW,0);
  steering = robotstop;
  Serial.println("i stopped");
}

void goBackwards(){
 setL(HIGH,255);
 setR(HIGH,255); 
 steering = backwards;
}
void kill(){
  robot_stop();
}

