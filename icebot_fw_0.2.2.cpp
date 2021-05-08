#include <Arduino.h>
#include <Encoder.h>
#include <PID_v1.h>

//initialize encoders
Encoder myEnc(2, 3); // 20.75 Pulses per mm

//Initialize Limit Switch
#define h_limit 21

// Initialize start Button
#define start_btn 7

//Initialize Motor
#define motor_fwd 5 //Motor Forward PWM Pin
#define motor_rev 6 //Motor Reverse PWM Pin

//Set Up the Encoder Variables
float ppmm = 20.6;  //Pulses per mm of the carriage
int encState = 1;   //Default State of the Encoder STate machine
long newPos = 0;    //New Encoder Position
long oldPos = -999; //Old encoder position

//Limit Switch Setup
int PrevHState = 0;    //Previous state
int hstate = 1;         //variable for the state machine of the homing state machine
int homingStatus = 0;  //0 indicates not homed 1 indicates homed
int homing_speed = 110; //speed of motor when homing
int hLimVal = 0;      //Limit Switch value

String message = "Placeholder";     //for storing debug messages

//Maestro State Machine Variables
int maestroState = 0; //Base state of the Maestro

//Movement State machine variables
int movState = 6;                          //State of the movement State Machine
long targetPos = 100;                      //current target position
long targetArray[] = {213, 480, 300, 747}; //Array that stores the target position in mm
int programLength = 3;                     //stores the length of the program
int arrayIndex = 0;                        //stores the current index of the array target position
long gap = 0;                              //gap between the target and the current position

int arvl_state = 0;     //state of the arrival state machine. If the motor has arrived within a certain tolerance, it switches state. 0 means it hasn't arrived, 1 means it has arrived
int arrival_status = 0; //status of the carriage. Zero for out of position, 1 for in position

//PID Setup
double Setpoint, Input, Output; //Define Variables we'll be connecting to

PID myPID(&Input, &Output, &Setpoint, 0.680, 0.09, 0.018, DIRECT); //Specify the links and initial tuning parameters

//motor setups
int mot_speed = 0; //Stores the speed of the motor (PWM value from 0-255)

void setup()
{

    //Set Up Motors
    pinMode(motor_fwd, OUTPUT);
    pinMode(motor_rev, OUTPUT);

    //Set up inputs
    pinMode(start_btn, INPUT_PULLUP);
    pinMode(h_limit, INPUT_PULLUP);

    //turn the PID on
    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(1);           // refresh rate of PID controller
    myPID.SetOutputLimits(-250, 250); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.

    //Enable Communication
    Serial.begin(9600);
    Serial.println("Connected!");
}

void loop() // put your main code here, to run repeatedly:
{
    read_enc();
    maestro();
}

//The master controller. The Maestro, if you will
void maestro(){
    switch (maestroState)
    {
        //Not Homed State
        case 0:
            if(homingStatus == 0)
            {
                homing_sm();    //Trigger the homing procedure if the machine is not homed
            }else
            {
                maestroState = 1;
            }
        break;

        //Homed State
        case 1:
            Serial.println("Homing Complete");
            maestroState = 2;
        break;

        //Ready State - Check for destination
        case 2:
          get_target();
          arrayIndex++;
          message = "Target has been set to " + String(targetPos) + "mm. Beginning PID calculation";
          Serial.println(message);
          targetPos = from_mm(targetPos);
          maestroState = 3;
        break;

        //calculate the PID
        case 3:
        check_gap();
        movState = 1;
        Setpoint = targetPos;
        Input = newPos;
        myPID.Compute();
        mot_speed = Output;
        mov_sm();

        
        


        break;
    
    default:
        break;
    }
    
}

//Converts mm to pulses by receiving a target in mm
int from_mm(long mm)
{
    int conversion;
    mm = mm - 90;                  //)ffset to center the cup under the dispenser. Subtract 90 mm to cneter target to the center of the cupholder
    conversion = round(mm * ppmm); //Multiply the mm by the encoder resolution
    return conversion;
}

void get_target(){
  if(arrayIndex <= programLength){
    targetPos = targetArray[arrayIndex];
  }else{
    motstop();
  }
}

//Reads the encoder and prints out the position
void read_enc()
{
    newPos = myEnc.read();
    if (newPos != oldPos)
    {
        oldPos = newPos;
        //Serial.print("Current Encoder Position:");
        Serial.println(newPos);
    }
}

//State machine for the homing procedure
void homing_sm()
{
    mot_speed = homing_speed;
    reverse();
    hLimVal = digitalRead(h_limit);
    if (hLimVal == 1)
    {
        motstop();
        delay(180);
        myEnc.write(0);
        Serial.println("Homing complete; Encoder set to Zero!");
        delay(10);
        message = "Current encoder value: " + String(newPos);
        Serial.println(message);
        homingStatus = 1;
        message = "Current Homing Status: " +String(homingStatus);
    }else{
        homingStatus = 0;
    }
}

void check_gap(){
    gap = abs(targetPos - newPos);
    message = "Distance to target is: "+ String(gap);
    Serial.println(message);
    if (gap < 20)
    {
        Serial.println("Arrived at Destination! ");
        Serial.print(int(gap));
        Serial.println("Disabling Motor");
        arrayIndex++;
        motstop();
        movState = 0;
        arrival_status = 1;
    }
}

//State machine that handles carriage movement when not homing
void mov_sm()
{
    oldPos = newPos;
    switch (movState)
    {
    //Does nothing; the default state
    case 0:
        movState = 1;
        break;
    case 1:
        //convert from mm to encoder position
        
        if(mot_speed > 0){
          forward();
        }else{
          reverse();
        }
        break;
    case 3:
        forward();
        movState = 5;
        break;

    case 4:
        reverse();
        Serial.println("Reverse");
        movState = 5;
        break;

    case 6:
        break;
    }
}

//Moves Motor forward
void forward()
{
    analogWrite(motor_fwd, mot_speed);
    digitalWrite(motor_rev, LOW);
}

//Reverses Motor Direction
void reverse()
{
    analogWrite(motor_rev, mot_speed);
    digitalWrite(motor_fwd, LOW);
}

//Disables the motor power
void motstop()
{
    digitalWrite(motor_rev, LOW);
    digitalWrite(motor_fwd, LOW);
    Serial.println("Motor Disabled");
}
