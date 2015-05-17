
/******************************************************************
Design Competition 2015 Code
Eric Elias, Paul Green, Jane Miller, and Christopher Pontisakos
******************************************************************/
//RGB LED fil
#include <Adafruit_NeoPixel.h>
#include <avr/power.h>

#define PIN            8
#define NUMPIXELS      1
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_RGB + NEO_KHZ800);



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
Break Beam
*************************/
//Laser/phototransistor in front of the roller
//If it is low (broken), something is being pushed
//I am attempting to program an interrupt that acts based on  
//when there is a change in the state of the pin
//See: http://www.arduino.cc/en/Reference/AttachInterrupt

//interrupt 0 is on digital pin 2
const int breakPin = 0;

/*A variable should be declared volatile whenever its value can be changed by something beyond the control of 
the code section in which it appears, such as a concurrently executing thread. 
In the Arduino, the only place that this is likely to occur is in sections of code associated with interrupts. */

volatile int state = LOW;

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


/********************************
Test Stuff
********************************/
int numColorChange;

void setup(){
  
  //test post plz ignore
  numColorChange = 0;

  
  //RGB LED (may not be needed)
  pixels.begin(); // This initializes the NeoPixel library.
  
  //initializes the HBridge pins
  int i = 0;
  for (i=0; i <= 3; i++){
    pinMode(hbridgepins[i], OUTPUT);
  }
  
  //Color sensor moving average
  colorSum = 0;
  colorPos = 0;
  for (i = 0; i < 10; i++){
    isOnWhite = colorCheck();
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
  
  //breakPin interrupt stuff
  pinMode(13, OUTPUT);
  attachInterrupt(breakPin, test, CHANGE);
  
  Serial.begin(9600);  
  
}

void loop() {
  //setLED();
  /*goStraight();
  isOnWhite = colorCheck();
  if(isOnWhite){
     digitalWrite(ledPin, LOW);
  }
  else{
    digitalWrite(ledPin,HIGH);
  }
  if(currColor != oldColor){
    goLeft();
    delay(500);
  }*/
 goLeft();
  digitalWrite(13,state);
  
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
void goStraight(){
  setL(LOW,255);
  setR(LOW,255);
}

void goLeft(){
  setL(LOW,0);
  setR(LOW,255);
}

void goRight(){
  setL(LOW,255);
  setR(LOW,0);
}

void robot_stop(){
  setL(LOW,0);
  setR(LOW,0);
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

void test(){
  state = !state;
  
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
