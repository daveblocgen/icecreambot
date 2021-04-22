#include <Arduino.h>
String inByte;
String payLoad;



void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
}

void funcOne(){
    payLoad = Serial.readStringUntil('}');
    if(payLoad == "ON"){
      digitalWrite(LED_BUILTIN, HIGH);
    }
    Serial.println("Hello there Sir! Your LED is now " + payLoad);
}

void funcTwo(){
    payLoad = Serial.readStringUntil('}');
    if(payLoad == "OFF"){
      digitalWrite(LED_BUILTIN, HIGH);
    }
    Serial.println("Hello there Sir! Your LED is now " + payLoad);
}

void loop() {
  if (Serial.available() > 0) {
    inByte = Serial.readStringUntil(':');
    // Serial.print(inByte);
    if(inByte == "Potato"){
      funcOne();
    } else if(inByte == "Banana"){
      funcTwo();
    }else
    {
      Serial.println("No clue what you're saying bro");
    }
    
  }
}
