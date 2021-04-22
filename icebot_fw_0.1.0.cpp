#include <Arduino.h>
#include <Encoder.h>

//initialize encoders
Encoder myEnc(2,3);

//Initialize Limit Switch
#define h_limit 23

// Initialize start Button
#define start_btn 7

//Initialize Motor
#define motor_fwd 5
#define motor_rev 6

//Encoder Setup
int encState = 1;
long newPos = 0;           //New Encoder Position
long oldPos = -999;        //Old encoder position

//Limit Switch Setup
int prev_hstate = 0;        
int hstate = 1;             //
int homing_status = 0;      //0 indicates not homed 1 indicates homed
int h_lim_val = 0;          //Limit Switch value

//motor setups
int mot_speed = 0;

void setup() {
  
  pinMode(start_btn, INPUT_PULLUP);
  pinMode(h_limit, INPUT_PULLUP);

  Serial.begin(9600);  
  Serial.println("Connected!");
}

void loop() {
  // put your main code here, to run repeatedly:
  enc_sm();
  homing_sm();

  // if(hstate == 3){
  //   delay(1000);
  //   encState = 2;
  //   newPos = 9000;
  //   Serial.print("Homeded!!!!");
  //   Serial.println(newPos);
  //   delay(30);
  //   encState = 0;
  //   hstate = 0;
  // }
}

//State Machine for reading the encoder position
void enc_sm(){
  
  switch (encState) {
    
    //Standby State
    case 0:
      encState = 1;
    break;

    //Encoder Armed and reading
    case 1:
    newPos = myEnc.read();
    if(newPos != oldPos){
      oldPos = newPos;
      Serial.print("Current Encoder Position:");
      Serial.println(newPos);      
      encState = 2;
    }
    break;

    //Homing Complete - Set encoder to Zero
    case 2:
      if(hstate == 3){
        delay(150);
        myEnc.write(0);
        Serial.println(" Homing complete; Encoder set to Zero");
        encState = 1;
      }
      encState = 1;
    break;
  }
}

void reverse() {
    analogWrite(motor_rev, mot_speed);
    digitalWrite(motor_fwd, LOW);
}

void motstop(){
  digitalWrite(motor_rev, LOW);
  digitalWrite(motor_fwd, LOW);
}


void homing_sm() {
    prev_hstate = hstate;
    mot_speed = 70;

    switch (hstate) {
        case 0:     //reset
        break;

        case 1:     //Standby State
          Serial.println(hstate);
          hstate = 2;
        break;

        case 2:     //Homing - Begin travelling in reverse
          reverse();
            
          h_lim_val = digitalRead(h_limit);
            //Serial.println(h_lim_val);

          if(h_lim_val == 1){            
            Serial.println("Limit Switch has been hit!!!!");
            //Serial.println(h_lim_val);
            motstop();
            hstate = 3;
          }
            
        break;

        case 3:     //Transitory State
          hstate = 4;
        break;

        // Error State
        case 4:
        break;
    }


}

