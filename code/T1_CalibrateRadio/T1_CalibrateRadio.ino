//#define CALIBRATE

#define PIN_M1  8
#define PIN_M2  9
#define PIN_M3 10
#define PIN_M4 11

#define PIN_ROLL      2
#define PIN_PITCH     3
#define PIN_YAW       5
#define PIN_THROTTLE  4
#define PIN_AUX1      6
#define PIN_AUX2      7

#define MAX_ANGLE_REF_DEG 15.0f

#include "Servo_200Hz.h"

Servo M1;
Servo M2;
Servo M3;
Servo M4;

#include "FastRCReader.h"

RCChannelMapper radio;

#include "Fsm.h"

State state_disarmed(&on_enter_disarmed, &on_state_disarmed, NULL);
State state_flight(&on_enter_flight, &on_state_flight, NULL);
State state_flight_failsafe(&on_enter_failsafe, NULL, NULL);
State state_error(&on_enter_error, NULL, NULL);
Fsm fsm(&state_disarmed);

#define EVENT_DISARMING 0
#define EVENT_ARMING 1
#define EVENT_ERROR 2

void on_enter_disarmed()
{
  
}

void on_state_disarmed()
{
#ifdef CALIBRATE
  while(1)
  {
    Serial.print(radio.getFreq(PIN_ROLL));
    Serial.print("\t");
    Serial.print(radio.getFreq(PIN_PITCH));
    Serial.print("\t");
    Serial.print(radio.getFreq(PIN_YAW));
    Serial.print("\t");
    Serial.print(radio.getFreq(PIN_THROTTLE));
    Serial.print("\t");
    Serial.print(radio.getFreq(PIN_AUX1));
    Serial.println();
    delay(50);
  }
#else
  while(1)
  {
    Serial.print(radio.getChannel(PIN_ROLL));
    Serial.print("\t");
    Serial.print(radio.getChannel(PIN_PITCH));
    Serial.print("\t");
    Serial.print(radio.getChannel(PIN_YAW));
    Serial.print("\t");
    Serial.print(radio.getChannel(PIN_THROTTLE));
    Serial.print("\t");
    Serial.print(radio.getChannel(PIN_AUX1));
    Serial.println();
    delay(50);
  }
#endif
}

void on_enter_flight()
{
  
}

void on_state_flight() 
{
  
}

void on_enter_failsafe()
{
  
}

void on_enter_error()
{

  
}

void setup()
{
  Serial.begin(115200);
  
  fsm.add_transition(&state_disarmed, &state_flight, EVENT_ARMING, NULL);
  fsm.add_transition(&state_flight, &state_disarmed, EVENT_DISARMING, NULL);

  if(M1.attach(PIN_M1) == 0) fsm.trigger(EVENT_ERROR);
  if(M2.attach(PIN_M2) == 0) fsm.trigger(EVENT_ERROR);
  if(M3.attach(PIN_M3) == 0) fsm.trigger(EVENT_ERROR);
  if(M4.attach(PIN_M4) == 0) fsm.trigger(EVENT_ERROR);

  radio.begin();

  for(int i = 2; i < 8; i++) radio.addChannel(i); // PIN D2 al D7

  radio.setMap(1010, 1536, 1990, PIN_ROLL);
  radio.setMap(1010, 1480, 1986, PIN_PITCH);
  radio.setMap(1010, 1536, 1990, PIN_YAW);
  radio.setMap(1010, 1060, 1990, PIN_THROTTLE);
  radio.setMap(1010, 1480, 1990, PIN_AUX1, 1, 2, 3);

  fsm.run_machine();
}

void loop(){}
