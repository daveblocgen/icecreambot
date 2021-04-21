#include <Arduino.h>

//initialize encoders
#define encA 2
#define encB 3

int encState = 0;
int prevEncB = 1;       //previous encoder B
int curEncB = 0;        //Current encoder B
int curEncA = 0;
long encPos = 1000;

//Direction Variable
int dir = 0;            //0 is clockwise, 1 is counterclockwise


void setup() {
  // put your setup code here, to run once:
  pinMode(encA, INPUT);
  pinMode(encB, INPUT);

  Serial.begin(9600);
}

void enc_sm(){

  prevEncB = curEncB;
  switch(encState){
    //reset encoder position to 0
    case 1:   
      encPos = 0;
      encState = 2;
      
    break;
    
    //Check to see if encoder B has changed state
    case 2:   
      curEncB = digitalRead(encB);

      if(curEncB != prevEncB){
        curEncA = digitalRead(encA);
        encState = 3;

      }
    break;

    //Check if going Clockwise or Counterclockwise
    case 3:
    if(curEncA == curEncB){
      encState = 4;       //clockwise
    }else{
      encState = 5;       //counterclockwise
    }
    break;

    //Motor is going Clockwise
    case 4:
      dir = 0;
      encPos += 1;        //increment position by 1
      Serial.print("Moving Clockwise, current position: ");
      Serial.println(encPos);
      encState = 2;
    break;

    //Motor is going Counterclockwise
    case 5:
      dir = 1;
      encPos -= 1;        //decrement position by 1
      Serial.print("Moving CounterClockwise, current position: ");
      Serial.println(encPos);
      encState = 2;
    break;

  }
}

void loop() {
  // put your main code here, to run repeatedly:
  enc_sm();
  if(encState == 4){
    
  }
}

