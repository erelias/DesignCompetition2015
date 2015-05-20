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




//RGB LED
#define PIN            8
#define NUMPIXELS      1
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_RGB + NEO_KHZ800);

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
const int colorPin = 5;
//the threshold between purple and white
const int colorThreshold = 600;

//moving average
const int colorSize = 5;
int colorAvg[colorSize];
int colorPos;
int colorSum;
int colorVal;

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


/********************************
Encoder Stuff
********************************/
int leftEncoderPin = 21; //interrupt 2
int rightEncoderPin = 20; //interrupt 3
volatile int leftEncoder = 0;
volatile int rightEncoder = 0;

/*******************************
Timer
*******************************/
Timer t; //Sets up a timer to shut off the motors after 3 minutes


void setup(){
  
  
  //RGB LED (may not be needed)
  pixels.begin(); // This initializes the NeoPixel library.
  
  //initializes the HBridge pins
  int i = 0;
  for (i=0; i <= 3; i++){
    pinMode(hbridgepins[i], OUTPUT);
  }
  
  //Color sensor moving average + ultrasonic moving average
  colorSum = 0;
  colorPos = 0;
  fSonarPos = 0;
  fSonarSum = 0;
  for (i = 0; i < 10; i++){
    isOnWhite = colorCheck();
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
  
  //Encoder Interrupts
  attachInterrupt(2, leftEncode,FALLING);
  attachInterrupt(3, rightEncode, FALLING);
  
  //causes the robot to stop moving after 3 minutes (1000 millisec/sec * 60 sec/min * 3 min)
  t.every((long)1000*60*3,kill);
  
  goForward();
  robotState = prePlanned;
  
  Serial.begin(9600);  
  
  
}

void loop() {
 
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
  } 
  Serial.println(analogRead(leftRetroPin));
 /* Serial.print("Left Encoder: ");
  Serial.println(leftEncoder);
  Serial.print("Right Encoder: ");
  Serial.println(rightEncoder);
 */
 
 
 
  
}

void leftEncode(){
 leftEncoder++;
 //Serial.println(leftEncoder); 
}

void rightEncode(){
  rightEncoder++;
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


boolean colorCheck(){
  //Checks to see what color the robot is on
  //returns 1 if on white, 0 if is on purple
  colorSum -= colorAvg[colorPos];
  colorAvg[colorPos] = analogRead(colorPin);
  colorSum += colorAvg[colorPos];
  colorPos += 1;
  if (colorPos >= colorSize){
    colorPos = 0;  
  }
  oldColor = currColor;
  colorVal = colorSum/colorSize;
  if (colorVal <= colorThreshold){
    currColor = purple;
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

void setLED(){
    
    if (colorVal<600){
        // For a set of NeoPixels the first NeoPixel is 0, second is 1, all the way up to the count of pixels minus one.
      for(int i=0;i<NUMPIXELS;i++){

        // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
        pixels.setPixelColor(i, pixels.Color(200,0,255)); // Moderately bright green color.

       pixels.show(); // This sends the updated pixel color to the hardware.
       
      delay(10); // Delay for a period of time (in milliseconds).
      
    }}
    else{
         // For a set of NeoPixels the first NeoPixel is 0, second is 1, all the way up to the count of pixels minus one.
      for(int i=0;i<NUMPIXELS;i++){

        // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
        pixels.setPixelColor(i, pixels.Color(255,255,255)); // Moderately bright green color.

       pixels.show(); // This sends the updated pixel color to the hardware.
        
      delay(10); // Delay for a period of time (in milliseconds).
      }
    }
}

void kill(){
 robotState = killSwitch;
  
}
