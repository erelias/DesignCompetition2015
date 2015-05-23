/******************************************************************
This program makes the robot turn until it sees a block, then it stops in the direction it saw the block in
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



enum stateMachine {
  prePlanned,
  search,
  found,
  killSwitch,
  RandomBlocks,

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
Team number
*************************/
int teamColor =1;

/************************
Front Retroreflective Sensors
************************/
#define RETRO_THRESH            300
const int leftRetroPin = 15;
const int rightRetroPin = 0;
const int centerRetroPin = 2;
const int blockPin = 6;

/*************************
Color Sensor
*************************/
//analog pin for color sensor
const int frontColorPin = 5;
const int backColorPin = 4;
//the threshold between purple and white
const int colorThreshold = 450;
const int backThreshold = 450;

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
enum color {
  purple,
  white
};
int startColor;
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

int hbridgepins[] = {enableL, phaseL, enableR, phaseR};

enum driveDirection {
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


/*******************************
Timer
*******************************/
Timer t; //Sets up a timer to shut off the motors after 3 minutes

bool done; //tells test code to run or noth
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


//janes test variables
int randomDir;

void setup() {

  /////
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
  for (i = 0; i <= 3; i++) {
    pinMode(hbridgepins[i], OUTPUT);
  }

  //Color sensor moving average + ultrasonic moving average
  frontColorSum = 0;
  frontColorPos = 0;
  backColorSum = 0;
  backColorPos = 0;
  fSonarPos = 0;
  fSonarSum = 0;
  for (i = 0; i < 10; i++) {
    isOnWhite = frontColorCheck();
    backColorCheck();
    backColorCheck();
    frontSonarCheck();
  }
  //Tells us what color we started on
  //if we start on white, we want to go to purple
  if (isOnWhite) {
   
    startColor = 1;

  }
  else {

    startColor = 0;

  }
  
  /////////////////////////////////////////
  //teamColor=startColor;//////////////////
  /////////////////////////////////////////

  for (i = 0; i < 5; i++) {
    //breakArray[i] = state;
    Serial.println(fSonar.ping_cm());

  }
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);


  //causes the robot to stop moving after 3 minutes (1000 millisec/sec * 60 sec/min * 3 min)
  t.every((long)1000 * 60 * 3 - 15000, kill);

  leftSpeed = 220;//210;
  leftCounter = 0;
  rightSpeed = 200;//255;
  rightCounter = 0;

  //  goForward();
  robotState = prePlanned;
  

  Serial.begin(9600);
  randomSeed(analogRead(10));
  randomDir = random(2);


}

void loop() {
  t.update();
  int spinCheck;
  switch (robotState) {
    case prePlanned:
    /*  Serial.print("left retro: ");
      Serial.println(analogRead(leftRetroPin));
      Serial.print("right retro: ");
      Serial.println(analogRead(rightRetroPin));
      Serial.print("center retro: ");
      Serial.println(analogRead(centerRetroPin));
      Serial.print("break beam: ");
      Serial.println(analogRead(blockPin));
      Serial.print("front color: ");
      Serial.println(analogRead(frontColorPin));
      Serial.print("back color: ");
      Serial.println(analogRead(backColorPin));
      Serial.print("\n\n\n");
      delay(1000);*/

      forwardOneSquare();
      forwardOneSquare();
      forwardOneSquare();
      fullBack();
      delay(200);
      robotState=RandomBlocks;
      break;
    case RandomBlocks:
    t.update();
      //spin in a random direction until block is seen
     
  randomDir = random(2);
        
      
        if (randomDir == 1) {
          Serial.println("I decided to go right");
          //start with an initial turn amount
          //turn right until we see a block
          spinCheck = 0;
            while (seeBlock()==false){
              t.update();
              spinCheck+=1;
              goRight();
              if(spinCheck>4000){
                goLeft();
              }
              delay(5);
              robot_stop();
              delay(5);
              
            }
          }
         else {
           Serial.println("I decided to go left");
           //start with an initial turn amount
           //turn right until we see a block
           while (seeBlock()==false){
             t.update();
             spinCheck+=1;
             goLeft();
             if(spinCheck>4000){
               goRight();
             }
             delay(5);
             robot_stop();
             delay(5);
            }
          }
          robot_stop();
          Serial.println("Stopped because block seen");
       
         
          goToBlock();
          robot_stop();
          pushBlockForward();
          robot_stop();
          fullBack();
          robot_stop();
          t.update();
          break;
    case search:
      Serial.println(fSonar.ping_cm());
      delay(1000);
    case killSwitch:
      robot_stop();
      break;

  }

      
}

boolean seeBlock() {
  //int tempL=analogRead(leftRetroPin);
  //int tempR=analogRead(rightRetroPin);
  int tempC = analogRead(centerRetroPin);
  if (tempC>500){
  //if ((tempL > 500) || (tempR > 500)) {
    Serial.println("I saw a block");
    return true;
  }
  else {
    return false;
    Serial.println("no block :c");
  }
}
//Pre planned path code
//!!!!!!!!!!!!!!!!!!
//Battery Dependent
//!!!!!!!!!!!!!!!!!!

void leftNinety() {
  t.update();
  robot_stop();
  delay(2000);
  goLeft();
  delay(leftNinetyTime);
  robot_stop();

}

void rightNinety() {
  t.update();
  robot_stop();
  delay(1000);
  goRight();
  delay(rightNinetyTime);
  robot_stop();


}

void forwardOneSquare() {
  t.update();
  int frontColor;
  int countStall = 0;
  int countWall = 0;
  frontColor = frontColorCheck();
  int frontNew = frontColor;
  int backNew = frontColor;
  int forwardCheck = 0;
  goForward();
  while ((frontNew == frontColor) || (backNew == frontColor)) {
    t.update();
    frontNew = frontColorCheck();
    //Serial.print(frontNew);
    //Serial.print("\t");
    backNew = backColorCheck();
    //Serial.println(backNew);

    if (frontNew != backNew) {
      countStall += 1;
      if (countStall > 5000) {
        backOverBump();
        goForward();
        countStall = 0;
        Serial.println("Stall");
      }
    }
    /*if (frontSonarCheck()==true){
      halfback();
      rightNinety();
      return;
    }*/
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
void backOverBump() {
  t.update();
  int backover_frontcolor = frontColorCheck();
  int backover_backcolor = backColorCheck();
  goBackwards();
  int back_countStall = 0;
  int backup_direction = 0;
  while (backover_frontcolor != backover_backcolor) {
    t.update();
    backover_frontcolor = frontColorCheck();
    backover_backcolor = backColorCheck();
    back_countStall += 1;
    if (back_countStall > 5000) {
      if (backup_direction == 0) {
        goForward();
        backup_direction = 1;
        back_countStall = 0;
      }
      else {
        goBackwards();
        backup_direction = 0;
        back_countStall = 0;
      }
    }
  }
  delay(300);
}




void halfBack() {
  t.update();
  goBackwards();
  delay(halfBackTime);
  robot_stop();

}

void fullBack() {
  t.update();
  int frontColor;
  int countStall = 0;
  int countWall = 0;
  frontColor = backColorCheck();
  int frontNew = frontColor;
  int backNew = frontColor;
  int forwardCheck = 0;
  goBackwards();
  while ((frontNew == frontColor) || (backNew == frontColor)) {
    t.update();
    frontNew = frontColorCheck();
    //Serial.print(frontNew);
    //Serial.print("\t");
    backNew = backColorCheck();
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
void setL(int phase, int enable) {
  digitalWrite(phaseL, phase);
  analogWrite(enableL, enable);
}

void setR(int phase, int enable) {
  digitalWrite(phaseR, phase);
  analogWrite(enableR, enable);
}

//Direction Controls
void goForward() {
  setL(LOW, leftSpeed);
  setR(LOW, rightSpeed);
  steering = forward;
}

void goLeft() {
  setL(HIGH, 100);
  setR(LOW, 255);
  steering = left;

}

void goRight() {
  setL(LOW, 255);
  setR(HIGH, 100);
  steering = right;
}

void robot_stop() {
  setL(LOW, 0);
  setR(LOW, 0);
  steering = robotstop;

}

void goBackwards() {
  setL(HIGH, 255 - leftSpeed + 20);
  setR(HIGH, 255 - rightSpeed);
  steering = backwards;
}


boolean frontColorCheck() {
  t.update();
  //Checks to see what color the robot is on
  //returns 1 if on white, 0 if is on purple
 
  frontColorSum -= frontColorAvg[frontColorPos];
  frontColorAvg[frontColorPos] = analogRead(frontColorPin);
  frontColorSum += frontColorAvg[frontColorPos];
  frontColorPos += 1;
  if (frontColorPos >= colorSize) {
    frontColorPos = 0;
  }
  oldColor = currColor;
  frontColorVal = frontColorSum / colorSize;
 
  if (frontColorVal <= colorThreshold) {
    currColor = purple;
    
    return 0;

  }
  else {
    currColor = white;
    
    
    return 1;
  }

}

boolean backColorCheck() {
  t.update();
  //Checks to see what color the robot is on
  //returns 1 if on white, 0 if is on purple
 
  backColorSum -= backColorAvg[backColorPos];
  backColorAvg[backColorPos] = analogRead(backColorPin);
  backColorSum += backColorAvg[backColorPos];
  backColorPos += 1;
  if (backColorPos >= colorSize) {
    backColorPos = 0;

  }
  oldBackColor = backColor;
  backColorVal = backColorSum / colorSize;
  
  if (backColorVal <= backThreshold) {
    
    
    backColor = purple;
    if (backColor != oldBackColor) {
      backColorChange++;
    }
    return 0;

  }
  if (backColor != oldBackColor) {
    backColorChange++;
  }
 

  backColor = white;

  return 1;

}

//Uses the sonar to check if there's a wall in front of it
//returns true if there is an object within a certain distance
boolean frontSonarCheck() {
  fSonarSum -= fSonarArray[fSonarPos];
  int uS = fSonar.ping_cm();
  Serial.println(uS);
  if (uS == 0) {
    uS = 200;
  }
  fSonarArray[fSonarPos] = 0;
  if (uS <= WALL_LIMIT) {
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



void kill() {
  robotState = killSwitch;
  robot_stop();
  digitalWrite(13,HIGH);
  delay((long)10000000);

}

void goToBlock() {
  t.update();
  delay(100);
  int isBlock =0;
  int blockRead = analogRead(blockPin);
  int centerRead;
  int leftRead;
  int rightRead;
  int countStall = 0;
  int frontNew;
  int backNew;
  int wallCount = 0;
  goForward();
  while(isBlock==0){
    t.update();
    goForward();
    blockRead = analogRead(blockPin);
    centerRead = analogRead(centerRetroPin);
    leftRead = analogRead(leftRetroPin);
    rightRead = analogRead(rightRetroPin);
    frontNew = frontColorCheck();
    backNew=backColorCheck();
    if(blockRead<200){
      isBlock=1;
      robot_stop();
      digitalWrite(13,HIGH);
      Serial.println("Block Near Roller");
    }
    if(frontNew=backNew){
      wallCount+=1;
      if(wallCount>50000){
        rightNinety();
        rightNinety();
        fullBack();
        robot_stop();
      }
    }
    else{
      wallCount=0;
    }
    //if we lose contact with center pin
    if((centerRead<500)&&(isBlock==0)) {
      //wait for one of the side sensors to find it
      while((leftRead<500)&&(rightRead<500)&&(centerRead<500)&&(blockRead>200)) {
        t.update();
        //just keep swimming
        centerRead = analogRead(centerRetroPin);
        leftRead = analogRead(leftRetroPin);
        rightRead = analogRead(rightRetroPin);
        blockRead = analogRead(blockPin);
        frontNew = frontColorCheck();
        backNew = backColorCheck();
        goForward(); 
        if (frontNew != backNew) {
      countStall += 1;
      if (countStall > 5000) {
        backOverBump();
        goForward();
        countStall = 0;
        Serial.println("Stall");
      }
    }
    else{
     countStall=0;
    }
      }
      if(blockRead<=400){
        isBlock=1;
        robot_stop();
      }
      //if we hit the left sensor
      else if(leftRead>=500){
        Serial.println("hitting left sensor");
        robot_stop();
        delay(100);
        goLeft();
        
        //turn until the center hits it
        centerRead = analogRead(centerRetroPin);
        while(centerRead<500){
          t.update();
          Serial.println("Center read under 500");
          centerRead = analogRead(centerRetroPin);
          robot_stop();
          delay(5);
          goLeft();
          delay(5);
        }
        robot_stop();
        delay(300);
        goForward();
      }
      //if we hit the right sensor
      else if(rightRead>=500){
        robot_stop();
        goRight();
        //turn until the center hits it
        Serial.println("right pin high");
        while(centerRead<500){
          t.update();
          centerRead = analogRead(centerRetroPin);
          Serial.println(centerRead);
          robot_stop();
          delay(5);
          goRight();
          delay(5);
        }
        robot_stop();
        delay(300);
        goForward();
      }
      //if we somehow hit the center again
      else if(centerRead>=500){
        //all is good, man
        goForward();
      }
      //we might mess this up but like whatever
      else{
        goForward(); //idk let's just go fast 
      }
    }
    if (frontNew != backNew) {
      countStall += 1;
      if (countStall > 5000) {
        backOverBump();
        goForward();
        countStall = 0;
        Serial.println("Stall");
      }
    }
    else{
     countStall=0;
    }
  }
  goForward();
}

void pushBlockForward(){
  t.update();
  //call this once a block has been grabbed.
  //move forward until you register one color change
  //once you do, the next time you hit the correct color you WIN!!!
  int blockFwd_frontColor;
  int blockFwd_frontColorBase = frontColorCheck();
  int blockFwd_backColor;
  int blockFwd_backColorBase = backColorCheck();
  int blockFwdTest=0;
  int blockFwd_offFirst=0;
  int blockFwd_dontBeDumb = 0; //descriptive variable names are overrated
  int wallCount = 0;
  int countStall = 0;
  
  goForward();
  

  
  while(blockFwd_offFirst==0) {
    t.update();
    goForward();
    blockFwd_frontColor=frontColorCheck();
    blockFwd_backColor=backColorCheck();
    //if we think that one of the colors have changed
    if((blockFwd_frontColor!=blockFwd_frontColorBase)||(blockFwd_backColor!=blockFwd_backColorBase)){
      blockFwd_dontBeDumb+=1;
      if(blockFwd_dontBeDumb==3){
        //hey cool we were right
        blockFwd_offFirst=1;
      }
    }
    else{
      //aww man we aren't there yet
      blockFwd_dontBeDumb=0;
    }
    if (blockFwd_frontColor != blockFwd_backColor) {
      countStall += 1;
      if (countStall > 5000) {
        backOverBump();
        goForward();
        countStall = 0;
        Serial.println("Stall");
      }
    }
    else{
     countStall=0;
    }
    //we still need code here to make sure we don't just hit a wall because this could run forever
    if(blockFwd_frontColor=blockFwd_backColor){
      wallCount+=1;
      if(wallCount>50000){
        rightNinety();
        rightNinety();
        fullBack();
        blockFwd_offFirst=1;
        blockFwdTest=1;
        return;
      }
    }
    else{
      wallCount=0;
    }
  }
  //go until we hit the start of a new square
  while(blockFwdTest==0){
    t.update();
    blockFwd_frontColor=frontColorCheck();
    if(currColor != teamColor){
      blockFwdTest=1;
    }
  }
  robot_stop(); //gotta go slow
}
