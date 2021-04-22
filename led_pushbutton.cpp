//pin definitions
#define btnLedPin 22
#define btn1Pin 23
 3

// Top Level Variables
int DEBUG = 1; // set debug to 1 to enable serial monitor debug info

//Pusbutton Variables
int state_btn1 = 0;                     //The Actual State of the button
int prev_btn1_state = 0;                      // Previous state of button 1
int val_btn1 = 0;                       //Value of Btn1 ("HIGH" or "LOW")
unsigned long t_btn1 = 0;               //The current time of the button
unsigned long t_0_btn1 = 0;             //The time since we last had a state of interest
unsigned long bounce_delay = 800;

//LED Status variables
int led_state = 0;                      //current state of the LED
int prev_led_state = 0;             //previous state of the LED
int val_led = 0;                        //LED Value (On or Off)
unsigned long t_btn_led = 0;            //Current Time of the LED button (in milliseconds)
unsigned long t_0_btn_led = 0;
unsigned long on_btn_led_delay = 800;   //On time for the button LED
unsigned long off_btn_led_delay = 800;  //Off time for the button LED
int blink_count_btn_led = 0;            //current number of times we've blinked
int blink_number_btn_led = 4;           //Number of times the LED should blink before resetting


void setup() {
  // put your setup code here, to run once:
  pinMode(btn1Pin, INPUT_PULLUP);
  pinMode(btnLedPin, OUTPUT);

  if(DEBUG){
    Serial.begin(9600);
    Serial.println("Debugging is on");
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  btn_sm();
  led_sm();

  if(state_btn1 == 4){
    Serial.println("Button has been pushed");
    led_state = 1;
  }
  

  if(state_btn1 != prev_btn1_state){
    Serial.print("Button State = : ");
    Serial.println(state_btn1);
  }

}


void btn_sm()
{
  prev_btn1_state = state_btn1;

  switch(state_btn1){
    case 0: //RESET
      state_btn1 = 1;
    break;

    case 1: //START
      val_btn1 = digitalRead(btn1Pin);
      if(val_btn1 == LOW){
        state_btn1 = 2;
      }
    break;

    case 2: //count
      t_0_btn1 = millis();
      state_btn1 = 3;
    break;

    case 3: //Wait
      val_btn1 = digitalRead(btn1Pin);
      t_btn1 = millis();

      
       if(val_btn1 == HIGH) //wait
      {
        state_btn1 = 0;
      }
      if(t_btn1 - t_0_btn1 > bounce_delay){
        Serial.println(t_btn1-t_0_btn1);
        state_btn1 = 5;
      }
    break;
    
    case 4: //TRIGGERED
      state_btn1 = 0;
    break;

    case 5: //armed
      val_btn1 = digitalRead(btn1Pin);
      if(val_btn1 == HIGH){
        state_btn1 = 4;
      }
  }

}

void led_sm()
{
  prev_led_state = led_state;
  switch(led_state){
    case 0: //Wait
    break;
    
    case 1: //turn on LED
      t_0_btn_led = millis();
      val_led = 1;
      digitalWrite(btnLedPin, val_led);
      led_state = 2;
    break;

    case 2:
    val_led = 0;
    t_btn_led = 0;
    t_btn_led = millis();
    if(t_btn_led - t_0_btn_led > on_btn_led_delay){
        led_state = 3;
      }
    
    break;

    case 3: //start new timer
      t_0_btn_led = millis();
      val_led = 0;
      digitalWrite(btnLedPin, val_led);
      led_state = 4;
    break;

    case 4: //
    t_btn_led = 0;
    t_btn_led = millis();
    if(t_btn_led - t_0_btn_led > on_btn_led_delay){
        led_state = 5;
      }
    break;

    case 5:
      digitalWrite(btnLedPin, val_led);
      led_state = 1;
    break;

  }
}
