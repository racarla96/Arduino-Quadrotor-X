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

#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

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

// PID
float max_throttle = 800; // 80% of the hole action control
float max_u = 200; // 20% of the hole action control

float Kp = 1;
float Ki = 1;
float Kt = 1;
float Kd = 1;

float ROLL_E = 0;
float ROLL_SUM_E = 0;
float PITCH_E = 0;
float PITCH_SUM_E = 0;

float ROLL_U = 0;
float PITCH_U = 0;
float THROTTLE_U = 0;

float ROLL_REF = 0;
float PITCH_REF = 0;

float M1_U = 0;
float M2_U = 0;
float M3_U = 0;
float M4_U = 0;
float BIAS_U = 1000;

float ROLL_SENS = 0;
float PITCH_SENS = 0;

float D_ROLL_SENS = 0;
float D_PITCH_SENS = 0;

void on_enter_disarmed()
{
  
}

void on_state_disarmed()
{
//  while(1)
//  {
//    Serial.print(radio.getChannel(PIN_ROLL));
//    Serial.print("\t");
//    Serial.print(radio.getChannel(PIN_PITCH));
//    Serial.print("\t");
//    Serial.print(radio.getChannel(PIN_YAW));
//    Serial.print("\t");
//    Serial.print(radio.getChannel(PIN_THROTTLE));
//    Serial.print("\t");
//    Serial.print(radio.getChannel(PIN_AUX1));
//    Serial.println();
//    delay(50);
//  }

  // PRIMERO CODIGO DE VUELO, SI FUNCIONA, DE LOCOS!
}

void on_enter_flight()
{
  ROLL_SUM_E = 0;
  PITCH_SUM_E = 0;
}

void on_state_flight() 
{
  while(1)
  {
    ROLL_REF = radio.getChannel(PIN_ROLL) * MAX_ANGLE_REF_DEG;
    PITCH_REF = radio.getChannel(PIN_PITCH) * MAX_ANGLE_REF_DEG;
  
    ROLL_SENS = mpu6050.getAngleX();
    PITCH_SENS = mpu6050.getAngleY();
  
    D_ROLL_SENS = mpu6050.getGyroX();
    D_PITCH_SENS = mpu6050.getGyroY();
  
    THROTTLE_U = radio.getChannel(PIN_THROTTLE) * max_throttle;
    if(THROTTLE_U < 0) THROTTLE_U = 0;
  
    ROLL_E = ROLL_REF - ROLL_SENS;
    PITCH_E = PITCH_REF - PITCH_SENS;
  
    ROLL_SUM_E += ROLL_E;
    PITCH_SUM_E += PITCH_E;
  
    ROLL_U = Kp * ROLL_E + Ki * ROLL_SUM_E - Kd * D_ROLL_SENS;
    PITCH_U = Kp * PITCH_E + Ki * PITCH_SUM_E - Kd * D_PITCH_SENS;
  
    M1_U = BIAS_U + ROLL_U + PITCH_U;
    M2_U = BIAS_U + ROLL_U + PITCH_U;
    M3_U = BIAS_U + ROLL_U + PITCH_U;
    M4_U = BIAS_U + ROLL_U + PITCH_U;
  
    // DE MOMENTO SIN ANTIWINDUP Y SIN TRACKING MODE
    
    M1.writeMicroseconds(M1_U);
    M2.writeMicroseconds(M2_U);
    M3.writeMicroseconds(M3_U);
    M4.writeMicroseconds(M4_U);
  }
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

  Wire.begin();
  mpu6050.begin();
  
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
