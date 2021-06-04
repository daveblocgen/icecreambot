#include <Arduino.h>
#include <Encoder.h>
#include <PID_v1.h>
#include <Servo.h>

//initialize encoders
Encoder myEnc(2, 3); // 20.6 Pulses per mm

//Initialize Limit Switch
#define h_limit 21

// Initialize start Button
#define start_btn 7

//Initialize Servo
Servo dispenseServo[3];
int activeServo = 0;    //Stores the active servo
int servoPosition[] = {225, 476, 749};
//Serial Communication variables
String inByte;
String payload;
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
int homing_speed = 150; //speed of motor when homing
int hLimVal = 0;      //Limit Switch value

String message = "Placeholder";     //for storing debug messages
int debug = 1;        //state machine debug
int encoderDebug = 0;   //shows Encoder values

//Maestro State Machine Variables
int maestroState = 0; //Base state of the Maestro
int oldMaestroState = 0;
int programComplete = 0;    //Set to 1 when the program is complete

//Movement State machine variables
int movState = 6;                          //State of the movement State Machine
int oldMovState = 0;                       //Previous movState
long targetPos = 100;                      //current target position
int targetArray[] = {749, 225, 476, 749, 225, 476, 900}; //Array that stores the target position in mm
int programLength = sizeof(targetArray) / sizeof(targetArray[0]);                   //stores the length of the program
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
  myPID.SetOutputLimits(-120, 170); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.

  for (int i = 0; i < 3; i++) {
    dispenseServo[i].attach(i + 8);
  }

  //Enable Communication
  Serial.begin(115200);
  Serial.println("Connected!");
}

void loop() // put your main code here, to run repeatedly:
{

  read_enc();
  maestro();
  serial_comm();

  //  dispenseServo.write(60);

  if (debug == 1) {
    if (maestroState != oldMaestroState || movState != oldMovState) {
      message = "Maestro State:" + String(maestroState) + "  movState:" + String(movState)
                + "  Homing Status: " + String(homingStatus) + "  ArrayIndex: " + String(arrayIndex) + "  Program Complete: " + String(programComplete);
      Serial.println(message);
      oldMaestroState = maestroState;
      oldMovState = movState;
    }
  }
}

void serial_comm() {
  int x, y, bs1, bs2;
  x = y = bs1 = bs2 = 0;
  if (Serial.available() > 0) {
    x = Serial.parseInt();
    y = Serial.parseInt();
    bs1 = Serial.parseInt();
    bs2 = Serial.parseInt();
    char r = Serial.read();
    if (r == '\n') {}

    Serial.print("x =  ");
    Serial.println(x);
    Serial.print("y =  ");
    Serial.println(y);
    Serial.print("bs1 =  ");
    Serial.println(bs1);
    Serial.print("bs2 =  ");
    Serial.println(bs2);
  }
}

//The master controller. The Maestro, if you will
void maestro() {
  switch (maestroState)
  {
    //Not Homed State
    case 0:
      if (homingStatus == 0)
      {
        homing_sm();
      } else
      {
        maestroState = 1;
      }
      break;

    //Homed State
    case 1:
      Serial.println("Homing Complete");
      if (programComplete == 0) {
        maestroState = 2;
      }
      break;

    //Ready State - Check for destination
    case 2:
      get_target();
      message = "Target has been set to " + String(targetPos) + "mm. Beginning PID calculation";
      Serial.println(message);
      targetPos = from_mm(targetPos);
      movState = 1;
      maestroState = 3;
      break;

    //calculate the PID
    case 3:

      if (arrival_status == 0) {
        check_gap();
        Setpoint = targetPos;
        Input = newPos;
        myPID.Compute();
        mot_speed = Output;
        mov_sm();
      } else {
        delay(200);
        message = "Maestro: Arrived at Destination! Encoder Position: " + String(newPos);
        open_dispensor(targetPos);
        Serial.println(message);
        maestroState = 4;
      }

      break;

    case 4:
      if (arrayIndex == programLength) {
        programComplete = 1;
        maestroState = 5;
      } else {
        arrival_status = 0;
        maestroState = 2;
      }
      break;

    case 5:
      //do nothing
      break;
  }

}

void open_dispensor(int carriage_destination) {

  if (carriage_destination == from_mm(servoPosition[0]))
  {
    dispenseServo[2].write(0);
    delay(400);
    dispenseServo[2].write(60);
    delay(400);
  }
  else if (carriage_destination == from_mm(servoPosition[1]))
  {
    dispenseServo[1].write(0);
    delay(400);
    dispenseServo[1].write(60);
    delay(400);
  }
  else if (carriage_destination == from_mm(servoPosition[2]))
  {
    dispenseServo[0].write(0);
    delay(400);
    dispenseServo[0].write(60);
    delay(400);
  }
}

//Converts mm to pulses by receiving a target in mm
int from_mm(long mm)
{
  int conversion;
  mm = mm - 90;                  //)ffset to center the cup under the dispenser. Subtract 90 mm to cneter target to the center of the cupholder
  conversion = round(mm * ppmm); //Multiply the mm by the encoder resolution
  message = "Encoder target: " + String(conversion);
  Serial.println(message);
  return conversion;
}

void get_target() {
  if (arrayIndex < programLength) {
    targetPos = targetArray[arrayIndex];
    Serial.println("target acquired from get_target");
    arrayIndex++;
  } else {
    Serial.println("Get Target stopping the motor");
    motstop();
    programComplete = 1;
    movState = 6;
  }
}

//Reads the encoder and prints out the position
void read_enc()
{
  newPos = myEnc.read();
  if (newPos != oldPos)
  {
    oldPos = newPos;
    if (encoderDebug == 1) {
      Serial.print("Current Encoder Position:");
      Serial.println(newPos);
    }
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
    maestroState = 1;
    message = "Current Homing Status: " + String(homingStatus);
  } else {
    homingStatus = 0;
  }
}

void check_gap() {
  gap = abs(targetPos - newPos);
  //message = "Distance to target is: " + String(gap);
  //Serial.println(message);
  if (gap < 20)
  {
    Serial.println("Arrived at Destination! ");
    Serial.print("Disabling Motor: Gap to Target is: ");
    Serial.print(int(gap));
    motstop();
    //    delay(300);
    movState = 6;
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
      if (mot_speed > 0) {
        forward();
      } else {
        reverse();
      }
      break;

    case 2:
      forward();
      //      if (mot_speed < 0) {
      movState = 1;
      //      }
      break;

    case 3:
      reverse();
      //      if (mot_speed > 0) {
      movState = 1;
      //      }
      break;

    case 6:
      break;
  }
}

//Moves Motor forward
void forward()
{
  digitalWrite(motor_rev, LOW);
  analogWrite(motor_fwd, mot_speed);
}

//Reverses Motor Direction
void reverse()
{
  digitalWrite(motor_fwd, LOW);
  analogWrite(motor_rev, mot_speed);
}

//Disables the motor power
void motstop()
{
  digitalWrite(motor_rev, LOW);
  digitalWrite(motor_fwd, LOW);
  //  Serial.println("Motor Disabled");
}
