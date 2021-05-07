#include <Arduino.h>
#include <Encoder.h>
#include <PID_v1.h>

//initialize encoders
Encoder myEnc(2,3);     // 20.75 Pulses per mm

//Initialize Limit Switch
#define h_limit 21

// Initialize start Button
#define start_btn 7

//Initialize Motor
#define motor_fwd 5
#define motor_rev 6

//Set Up the Encoder Variables
float ppmm = 20.6;          //Pulses per mm of the carriage
int encState = 1;          //Default State of the Encoder STate machine
long newPos = 0;           //New Encoder Position
long oldPos = -999;        //Old encoder position

//Limit Switch Setup
int prev_hstate = 0;        //Previous state
int hstate = 1;             //variable for the state machine of the homing state machine
int homing_status = 0;      //0 indicates not homed 1 indicates homed
int h_lim_val = 0;          //Limit Switch value

//Movement State machine variables
int movState = 0;           //State of the movement State Machine
int valOUT = 0;             //Value of the output from the PID
long targetPos = 100;      //current target position
long targetArray[] = {213, 480, 12000, 747};   //Array that stores the target position in mm
int arrayIndex = 3;         //stores the current index of the array target position
long gap = 0;               //gap between the target and the current position

int arvl_state = 0;         /*state of the arrival state machine. If the motor has 
                            arrived within a certain tolerance, it switches state*/
int arrival_status = 0;     //status of the carriage. Zero for out of position, 1 for in position

//PID Setup
double Setpoint, Input, Output;   //Define Variables we'll be connecting to

PID myPID(&Input, &Output, &Setpoint, 0.680, 0.09, 0.018, DIRECT);    //Specify the links and initial tuning parameters

//motor setups
int mot_speed = 0;            //Stores the speed of the motor (PWM value from 0-255)

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
  myPID.SetOutputLimits(-250, 250); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.

  //Enable Communication
  Serial.begin(9600);  
  Serial.println("Connected!");
}

void loop() // put your main code here, to run repeatedly:
{
  enc_sm();
  homing_sm();
  mov_sm();

}

//Converts mm to pulses by receiving a target in mm
int from_mm(long mm)
{
  int conversion;
  mm = mm - 90;       //Subtract 90 mm to cneter target to the center of the cupholder
  conversion = round(mm * ppmm);    //Multiply the mm by the encoder resolution

  return conversion;
}

//State Machine for reading the encoder position  
void enc_sm() 
{      
  switch (encState) {
    
    //Standby State
    case 0:
    break;

    //Encoder Armed and reading
    case 1:
    newPos = myEnc.read();
    if(newPos != oldPos){
      oldPos = newPos;
      //Serial.print("Current Encoder Position:");
      Serial.println(newPos);
      if(homing_status == 0){
        encState = 2;
      }else{
        encState = 3;
      }      
      
    }
    break;

    case 2:   //Homing Complete - Set encoder to Zero
      if(homing_status == 1){
        delay(150);
        myEnc.write(0);
        Serial.println(" Homing complete; Encoder set to Zero");
        Serial.println("target position is: ");
        Serial.print(targetPos);
        encState = 3;
        movState = 1;
        arvl_state = 1;
      }
      encState = 1;
    break;
   
    case 3:
      encState  = 1;
    break;
  }
}

//State machine for the homing procedure
void homing_sm()    
{
    prev_hstate = hstate;
    switch (hstate) {
        case 0:     //reset
        break;

        case 1:     //Standby State
          mot_speed = 130;
          if(homing_status == 0){
            Serial.println("Machine is not Homed. Initializing Homing.");
            hstate = 2;
          }else{
            hstate = 0;
          }
        break;

        case 2:     //Homing - Begin travelling in reverse
          reverse();
            
          h_lim_val = digitalRead(h_limit);
            //Serial.println(h_lim_val);

          if(h_lim_val == 1){            
            Serial.println("Homing Complete!");
            Serial.println(targetPos);
            //Serial.println(h_lim_val);
            motstop();
            homing_status = 1;
            hstate = 0;
          }else{
            hstate = 1;
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

//State machine that handles carriage movement when not homing
void mov_sm(){
  oldPos = newPos;
  switch(movState){
    //Does nothing; the default state
    case 0:
    break;

    case 1:
      targetPos = targetArray[arrayIndex];
      targetPos = from_mm(targetPos);
      Setpoint = targetPos;
      Input = newPos;
      myPID.Compute();
      mot_speed = Output;

      //check to see if the carriage has arrived
      if(arrival_status == 0){
        movState = 2; //checks the gap to the target if the carriage has not arrived
      }else{
        motstop();
        movState = 0;  //goes into standby if the target has been reached.
      }
    break;

    case 2:
      if(mot_speed > 0){
        movState = 3;
      }else{
        movState = 4;
      }
    case 3:
      forward();
      movState = 5;
    break;

    case 4:
      reverse();
      movState = 5;
    break;

    case 5:
      gap = abs(targetPos - newPos);
      //Serial.println("   Distance to target is: "+ gap);
      if(gap < 20){
        Serial.println("Arrived at Destination! ");
        Serial.print(int(gap));
        Serial.println("Disabling Motor");
        motstop();
        movState = 0;
        arrival_status = 1;
        
      }else{
        movState = 1;
        //Serial.println("Enabling Motor");
        arrival_status = 0;
      }
    break;
  }
}

//Moves Motor forward
void forward() {
    analogWrite(motor_fwd, mot_speed);
    digitalWrite(motor_rev, LOW);
}

//Reverses Motor Direction
void reverse() {
    analogWrite(motor_rev, mot_speed);
    digitalWrite(motor_fwd, LOW);
}

//Disables the motor power
void motstop(){
  digitalWrite(motor_rev, LOW);
  digitalWrite(motor_fwd, LOW);
}
