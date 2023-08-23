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
#define MAX_ANGLE_ERROR_DEG 20.0f

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
Fsm fsm(&state_flight);

#define EVENT_DISARMING 0
#define EVENT_ARMING 1
#define EVENT_ERROR 2

// PID
float max_throttle = 800; // 80% of the hole action control
float max_u = 200; // 20% of the hole action control

float Kp = 0.01;
float Ki = 0.000005; // 0.5
float Kt = 0;
float Kd = 0.000005;

float dt = 0.005;

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
float MIN_U = 1000;
float MAX_U = 2000;

float ROLL_SENS = 0;
float PITCH_SENS = 0;

float ROLL_SENS_BIAS = 0;
float PITCH_SENS_BIAS = 0;

float D_ROLL_SENS = 0;
float D_PITCH_SENS = 0;

float D_ROLL_SENS_BIAS = 0;
float D_PITCH_SENS_BIAS = 0;

unsigned long previousMicros = 0;  // Variable para almacenar el tiempo del último ciclo
const unsigned long intervalMicros = 5000;  // Intervalo en microsegundos para un bucle a 200 Hz (1000000 us / 200 Hz)

int radio_disconnected_counter = 0;
int radio_disconnected_max_counter = 20;

void on_enter_disarmed()
{
  M1.writeMicroseconds(MIN_U);
  M2.writeMicroseconds(MIN_U);
  M3.writeMicroseconds(MIN_U);
  M4.writeMicroseconds(MIN_U);
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
  // Problemon sino esta la radio conectada

  while (!radio.isRecvOn()) {}

  previousMicros = micros();
  while (1)
  {
    // Verificar si ha pasado el intervalo deseado
    while (micros() < previousMicros) {}
    previousMicros += intervalMicros;  // Actualizar el tiempo del último ciclo

    ROLL_REF = radio.getChannel(PIN_ROLL) * MAX_ANGLE_REF_DEG;
    PITCH_REF = radio.getChannel(PIN_PITCH) * MAX_ANGLE_REF_DEG;

    mpu6050.update();

    ROLL_SENS = -(mpu6050.getAngleX() - ROLL_SENS_BIAS);
    PITCH_SENS = -(mpu6050.getAngleY() - PITCH_SENS_BIAS);

    if(ROLL_SENS > MAX_ANGLE_ERROR_DEG) break;
    if(ROLL_SENS < -MAX_ANGLE_ERROR_DEG) break;
    if(PITCH_SENS > MAX_ANGLE_ERROR_DEG) break;
    if(PITCH_SENS < -MAX_ANGLE_ERROR_DEG) break;;
    
    D_ROLL_SENS = -(mpu6050.getGyroX() - D_ROLL_SENS_BIAS);
    D_PITCH_SENS = -(mpu6050.getGyroY() - D_PITCH_SENS_BIAS);

    THROTTLE_U = radio.getChannel(PIN_THROTTLE) * max_throttle;

    if (THROTTLE_U < 0) {
      THROTTLE_U = 0;
      ROLL_U = 0;
      PITCH_U = 0;
    }

    ROLL_E = ROLL_REF - ROLL_SENS;
    PITCH_E = PITCH_REF - PITCH_SENS;

    ROLL_SUM_E += ROLL_E * dt;
    PITCH_SUM_E += PITCH_E * dt;

    ROLL_U = Kp * ROLL_E + Ki * ROLL_SUM_E + Kd * D_ROLL_SENS;
    PITCH_U = Kp * PITCH_E + Ki * PITCH_SUM_E + Kd * D_PITCH_SENS;

    M1_U = MIN_U + THROTTLE_U + ROLL_U - PITCH_U;
    M2_U = MIN_U + THROTTLE_U - ROLL_U - PITCH_U;
    M3_U = MIN_U + THROTTLE_U - ROLL_U + PITCH_U;
    M4_U = MIN_U + THROTTLE_U + ROLL_U + PITCH_U;

    if (M1_U < MIN_U) M1_U = MIN_U;
    if (M1_U > MAX_U) M1_U = MAX_U;
    if (M2_U < MIN_U) M2_U = MIN_U;
    if (M2_U > MAX_U) M2_U = MAX_U;
    if (M3_U < MIN_U) M3_U = MIN_U;
    if (M3_U > MAX_U) M3_U = MAX_U;
    if (M4_U < MIN_U) M4_U = MIN_U;
    if (M4_U > MAX_U) M4_U = MAX_U;

    // DE MOMENTO SIN ANTIWINDUP Y SIN TRACKING MODE

    //    Serial.print(previousMicros);
    //    Serial.print("\t");
    //    Serial.print(THROTTLE_U);
    //    Serial.print("\t");
//    Serial.print(ROLL_SENS);
//    Serial.print("\t");
//    Serial.print(PITCH_SENS);
//    Serial.print("\t");
//    Serial.print(D_ROLL_SENS);
//    Serial.print("\t");
//    Serial.print(D_PITCH_SENS);
//    Serial.print("\t");
//    Serial.print(M1_U);
//    Serial.print("\t");
//    Serial.print(M2_U);
//    Serial.print("\t");
//    Serial.print(M3_U);
//    Serial.print("\t");
//    Serial.print(M4_U);
//    Serial.println();

    M1.writeMicroseconds(M1_U);
    M2.writeMicroseconds(M2_U);
    M3.writeMicroseconds(M3_U);
    M4.writeMicroseconds(M4_U);

    if(!radio.isRecvOn()) radio_disconnected_counter++;
    if(radio_disconnected_counter > radio_disconnected_max_counter) break; 
  }

  fsm.trigger(EVENT_DISARMING);
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

  if (M1.attach(PIN_M1) == 0) fsm.trigger(EVENT_ERROR);
  if (M2.attach(PIN_M2) == 0) fsm.trigger(EVENT_ERROR);
  if (M3.attach(PIN_M3) == 0) fsm.trigger(EVENT_ERROR);
  if (M4.attach(PIN_M4) == 0) fsm.trigger(EVENT_ERROR);

  radio.begin();

  for (int i = 2; i < 8; i++) radio.addChannel(i); // PIN D2 al D7

  radio.setMap(1010, 1536, 1990, PIN_ROLL);
  radio.setMap(1010, 1480, 1986, PIN_PITCH);
  radio.setMap(1010, 1536, 1990, PIN_YAW);
  radio.setMap(1010, 1060, 1990, PIN_THROTTLE);
  radio.setMap(1010, 1480, 1990, PIN_AUX1, 1, 2, 3);

  // CALIBRATION
  previousMicros = micros();
  for (int i = 0; i < 1000; i++)
  {
    // Verificar si ha pasado el intervalo deseado
    while (micros() < previousMicros) {}
    previousMicros += intervalMicros;  // Actualizar el tiempo del último ciclo

    mpu6050.update();

    ROLL_SENS_BIAS += mpu6050.getAngleX();
    PITCH_SENS_BIAS += mpu6050.getAngleY();

    D_ROLL_SENS_BIAS += mpu6050.getGyroX();
    D_PITCH_SENS_BIAS += mpu6050.getGyroY();
  }

  ROLL_SENS_BIAS /= 1000;
  PITCH_SENS_BIAS /= 1000;

  D_ROLL_SENS_BIAS /= 1000;
  D_PITCH_SENS_BIAS /= 1000;

  fsm.run_machine();
}

void loop() {}
