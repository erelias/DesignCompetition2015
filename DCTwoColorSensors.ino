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
#define WALL_LIMIT 10
NewPing fSonar(TRIGGER_FRONT, ECHO_FRONT, MAX_DIST_FRONT);

//Moving average
boolean fSonarArray[5];
int fSonarPos;
int fSonarSum;



/************************
Front Retroreflective Sensors
************************/
#define RETRO_THRESH            300
const int leftRetroPin = 15;
const int rightRetroPin = 0;

/*************************
Color Sensor 
*************************/
//analog pin for color sensor
const int frontColorPin = 5;
const int backColorPin = 4;
//the threshold between purple and white
const int colorThreshold = 400;
const int backThreshold = 550;

//moving average
const int colorSize = 5;
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
color backColor;
color oldBackColor;
color oldColor;


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
int backColorChange;
int fullBackTime;
int forwardOneTime;
int halfBackTime;
int leftNinetyTime;
int rightNinetyTime;

int backWhite = 50;
int backPurple = 48;
int frontWhite = 49;
int frontPurple = 51;
void setup(){
  
  pinMode(backWhite,OUTPUT);
  pinMode(backPurple,OUTPUT);
  pinMode(frontWhite, OUTPUT);
  pinMode(frontPurple,OUTPUT);  
  numColorChange = 0;
  backColorChange = 0;
  fullBackTime = 1150;
  forwardOneTime = 1500;
  halfBackTime = 400;
  leftNinetyTime = 640;
  rightNinetyTime = 650;
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
  t.every((long)1000*60*3-15000,kill);

  leftSpeed = 230;//240;
  leftCounter = 0;
  rightSpeed = 240;//255;
  rightCounter = 0;

 //  goForward();
  robotState = search;
  
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
  case search:
    Serial.println(fSonar.ping_cm());
    delay(1000);
  
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
       //rightNinety();
       forwardOneSquare();
       forwardOneSquare();
       forwardOneSquare();
       delay(100);
       fullBack();
       delay(100);
       rightNinety();
       delay(100);
       forwardOneSquare();
       //delay(100);
       forwardOneSquare();
       //delay(100);
       forwardOneSquare();
       delay(100);
       fullBack();
       delay(100);
       leftNinety();
       delay(100);
       forwardOneSquare();
       delay(100);
       leftNinety();
       delay(100);
       forwardOneSquare();
       //delay(100);
       forwardOneSquare();
       delay(100);
       fullBack();
       delay(100);
       leftNinety();
       delay(100);
       leftNinety();
       delay(100);
       forwardOneSquare();
       //delay(100);
       forwardOneSquare();
       //delay(100);
       forwardOneSquare();
       delay(100);
       fullBack();
       delay(100);
       rightNinety();
       delay(100);
       forwardOneSquare();
       //delay(100);
forwardOneSquare();

//delay(100);
forwardOneSquare();
//Serial.println("two squares");
//delay(100);
fullBack();
//delay(100);
leftNinety();
//delay(100);
forwardOneSquare();
//delay(100);
forwardOneSquare();
//delay(100);
forwardOneSquare();
//delay(100);
fullBack();
//delay(100);
rightNinety();
//delay(100);
forwardOneSquare();
//delay(100);
leftNinety();
//delay(100);
forwardOneSquare();
//delay(100);
forwardOneSquare();
//delay(100);
leftNinety();
//delay(100);
forwardOneSquare();
//delay(100);
forwardOneSquare();
//delay(100);
fullBack();
//delay(100);
fullBack();
//delay(100);
leftNinety();
//delay(100);
forwardOneSquare();
//delay(100);
forwardOneSquare();
//delay(100);
forwardOneSquare();
//delay(100);
forwardOneSquare();
robot_stop();

       /**rightNinety();
       forwardOneSquare();
       delay(100);
       rightNinety(); 
       **/
       
  //   }
    // }
    robotState=killSwitch;
    
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
  t.update();
robot_stop();
  delay(2000);
 goLeft();
 delay(leftNinetyTime);
 robot_stop();
  
}

void rightNinety(){
  t.update();
  robot_stop();
  delay(1000);
  goRight();
 delay(rightNinetyTime);
 robot_stop();
 
  
}

void forwardOneSquare(){
  t.update();
   int frontColor;
   int countStall=0;
   int countWall=0;
 frontColor=frontColorCheck();
  int frontNew=frontColor;
 int backNew=frontColor;
 int forwardCheck=0;
  goForward();
  while((frontNew==frontColor)||(backNew==frontColor)){
    frontNew = frontColorCheck();
    //Serial.print(frontNew);
    //Serial.print("\t");
    backNew=backColorCheck();
    //Serial.println(backNew);
    
    if(frontNew!=backNew){
      countStall+=1;
      if(countStall>2000){
        backOverBump();
        goForward(); 
        countStall=0;
        Serial.println("Stall");
      }
    }
    if (frontSonarCheck()==true){
      halfback();
      rightNinety();
      return;
    }
    /*if(frontNew==backNew){
      countWall+=1;
      if(countWall>2000){
        frontNew=!frontColor;
        backNew=!frontColor;
        goBackwards();
        delay(100);
        robot_stop();
        countWall=0;
      }
    }*/
    /*if(frontNew!=frontColor){
      if(backNew!=frontColor){
        forwardCheck=1;
      }
    }*/
   /* countWall+=1;
    if(countWall==3000){
      forwardCheck=1;
    }*/
  }
    robot_stop();
  
}
 void backOverBump(){
   int backover_frontcolor = frontColorCheck();
   int backover_backcolor = backColorCheck();
   goBackwards();
   int back_countStall=0;
   int backup_direction=0;
   while(backover_frontcolor!=backover_backcolor){
     backover_frontcolor=frontColorCheck();
     backover_backcolor=backColorCheck();
      back_countStall+=1;
      if(back_countStall>3000){
        if(backup_direction==0){ 
          goForward();
          backup_direction=1;
          back_countStall=0;
        }
        else{
          goBackwards();
          backup_direction=0;
        back_countStall=0;
        }
      }
   }
   delay(300);
 }
  
  


void halfBack(){
  goBackwards();
  delay(halfBackTime);
  robot_stop();
  
}

void fullBack(){
  t.update();
   int frontColor;
   int countStall=0;
   int countWall=0;
 frontColor=backColorCheck();
  int frontNew=frontColor;
 int backNew=frontColor;
 int forwardCheck=0;
  goBackwards();
  while((frontNew==frontColor)||(backNew==frontColor)){
    frontNew = frontColorCheck();
    //Serial.print(frontNew);
    //Serial.print("\t");
    backNew=backColorCheck();
    //Serial.println(backNew);
    
    /*if(frontNew!=backNew){
      countStall+=1;
      if(countStall>3000){
        backOverBump();
        goForward(); 
        countStall=0;
        Serial.println("Stall");
      }
    }
    if(frontNew==backNew){
      countWall+=1;
      if(countWall>2000){
        frontNew=!frontColor;
        backNew=!frontColor;
        goBackwards();
        delay(100);
        robot_stop();
        countWall=0;
      }
    }*/
    /*if(frontNew!=frontColor){
      if(backNew!=frontColor){
        forwardCheck=1;
      }
    }*/
   /* countWall+=1;
    if(countWall==3000){
      forwardCheck=1;
    }*/
  }
    robot_stop();
  
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
  setL(HIGH,75);
  setR(LOW,180);
  steering = left;
  
}

void goRight(){
  setL(LOW,180);
  setR(HIGH,75);
  steering = right;
}

void robot_stop(){
  setL(LOW,0);
  setR(LOW,0);
  steering = robotstop;
  
}

void goBackwards(){
 setL(HIGH,255-leftSpeed+20);
 setR(HIGH,255-rightSpeed); 
 steering = backwards;
}


boolean frontColorCheck(){
  //Checks to see what color the robot is on
  //returns 1 if on white, 0 if is on purple
  digitalWrite(frontWhite,LOW);
  digitalWrite(frontPurple,LOW);
  frontColorSum -= frontColorAvg[frontColorPos];
  frontColorAvg[frontColorPos] = analogRead(frontColorPin);
  frontColorSum += frontColorAvg[frontColorPos];
  frontColorPos += 1;
  if (frontColorPos >= colorSize){
    frontColorPos = 0;  
  }
  oldColor = currColor;
  frontColorVal = frontColorSum/colorSize;
  Serial.print("Front is on: ");
  if (frontColorVal <= colorThreshold){
    currColor = purple;
    digitalWrite(frontPurple,HIGH);   
    return 0;
    
  }
  else{
  currColor = white;
  digitalWrite(frontWhite,HIGH);
  Serial.println("white");
  return 1;
  }
  
}

boolean backColorCheck(){
  //Checks to see what color the robot is on
  //returns 1 if on white, 0 if is on purple
  digitalWrite(backWhite, LOW);
  digitalWrite(backPurple,LOW);
  backColorSum -= backColorAvg[backColorPos];
  backColorAvg[backColorPos] = analogRead(backColorPin);
  backColorSum += backColorAvg[backColorPos];
  backColorPos += 1;
  if (backColorPos >= colorSize){
    backColorPos = 0;  
    
  }
  oldBackColor = backColor;
  backColorVal = backColorSum/colorSize;
  Serial.print("Back is on: ");
  if (backColorVal <= backThreshold){
    digitalWrite(backPurple,HIGH);
    Serial.println("purple");
    backColor = purple;
    if(backColor != oldBackColor){
     backColorChange++; 
    }
    return 0;
    
  }
  if(backColor != oldBackColor){
     backColorChange++; 
    }
    Serial.println("white");
    digitalWrite(backWhite,HIGH);
  backColor = white;
  
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
     return true;
     //fSonarArray[fSonarPos] = 1; 
  }
  /*
  fSonarSum += fSonarArray[fSonarPos];
  fSonarPos += 1;
  if (fSonarPos >= 5){
    fSonarPos = 0;  
  }
  if(fSonarSum >= 4){
   return true; 
  }
  */
  return false;
}

void encoderReset(){
 leftEncoder = 0;
 rightEncoder = 0; 
  
}


void kill(){
 robotState = killSwitch;
  
}

