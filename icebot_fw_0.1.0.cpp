#include <Arduino.h>
#include <Encoder.h>
#include <PID_v1.h>

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
int hstate = 1;             //variable for the state machine of the
int homing_status = 0;      //0 indicates not homed 1 indicates homed
int h_lim_val = 0;          //Limit Switch value

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, 0.08, 0.009, 0.001, DIRECT);

int movState = 0;           //State of the movement State Machine
int valOUT = 0;             //Value of the output from the PID
long targetPos = 6000;

//motor setups
int mot_speed = 0;

void setup() {
  //Set Up Motors
  pinMode(motor_fwd, OUTPUT);
  pinMode(motor_rev, OUTPUT);

  //Set up inputs
  pinMode(start_btn, INPUT_PULLUP);
  pinMode(h_limit, INPUT_PULLUP);

    //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);  // refresh rate of PID controller
  myPID.SetOutputLimits(-255, 255); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.

  //Enable Communication
  Serial.begin(9600);  
  Serial.println("Connected!");
}

void loop() {
  // put your main code here, to run repeatedly:
  enc_sm();
  homing_sm();
  mov_sm();
}


void enc_sm(){
//State Machine for reading the encoder position  
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
        movState = 1;
      }
      encState = 1;
    break;
  }
}

void forward() {
    analogWrite(motor_fwd, mot_speed);
    digitalWrite(motor_rev, LOW);
}

void reverse() {
    analogWrite(motor_rev, mot_speed);
    digitalWrite(motor_fwd, LOW);
}

void motstop(){
  digitalWrite(motor_rev, LOW);
  digitalWrite(motor_fwd, LOW);
}

//PID movement state machine
void mov_sm(){
  switch(movState){
    //Does nothing; the default state
    case 0:
    break;

    case 1:
      
      Setpoint = targetPos;
      Input = newPos;
      myPID.Compute();
      mot_speed = Output;

      if(mot_speed > 0){
        movState = 2;
      }else{
        movState = 3;
      }
    break;

    case 2:
    forward();
    movState = 1;
    break;

    case 3:
    reverse();
    movState = 1;
    break;
  }
}

//State machine for the homing procedure
void homing_sm() {
    prev_hstate = hstate;
    

    switch (hstate) {
        case 0:     //reset
        break;

        case 1:     //Standby State
          mot_speed = 70;
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

