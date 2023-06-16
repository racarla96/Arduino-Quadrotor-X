#define DEBUG

//#define OUTPUT_READABLE_GYRO
//#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_YAWPITCHROLL

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"

#define MPU_INT_PIN 2
MPU6050 mpu;

#define LED_PIN LED_BUILTIN
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
//VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

int16_t gx, gy, gz;
float gxyz[3];
uint8_t g_scale;

//definitions for raw data
//gyro and acc scale
#define MPU6050_GYRO_FS MPU6050_GYRO_FS_250
#define MPU6050_ACCEL_FS MPU6050_ACCEL_FS_2

#define MPU6050_GYRO_LSB_250 1.0f/131.0f
#define MPU6050_GYRO_LSB_500 1.0f/65.5f
#define MPU6050_GYRO_LSB_1000 1.0f/32.8f
#define MPU6050_GYRO_LSB_2000 1.0f/16.4

#if MPU6050_GYRO_FS == MPU6050_GYRO_FS_250
#define MPU6050_G_GAIN MPU6050_GYRO_LSB_250
#elif MPU6050_GYRO_FS == MPU6050_GYRO_FS_500
#define MPU6050_G_GAIN MPU6050_GYRO_LSB_500
#elif MPU6050_GYRO_FS == MPU6050_GYRO_FS_1000
#define MPU6050_G_GAIN MPU6050_GYRO_LSB_1000
#elif MPU6050_GYRO_FS == MPU6050_GYRO_FS_2000
#define MPU6050_G_GAIN MPU6050_GYRO_LSB_2000
#endif

#define MPU6050_ACCEL_LSB_2 16384.0
#define MPU6050_ACCEL_LSB_4 8192.0
#define MPU6050_ACCEL_LSB_8 4096.0
#define MPU6050_ACCEL_LSB_16 2048.0
#if MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_2
#define MPU6050_AGAIN MPU6050_ACCEL_LSB_2
#elif MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_4
#define MPU6050_AGAIN MPU6050_ACCEL_LSB_4
#elif MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_8
#define MPU6050_AGAIN MPU6050_ACCEL_LSB_8
#elif MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_16
#define MPU6050_AGAIN MPU6050_ACCEL_LSB_16
#endif

// All time data in us
unsigned long t_clock;  // general task clock in us

unsigned long t_gyro;   // task gyro
unsigned long t_gyro_p = 5000; // task gyro period
unsigned long tp_gyro_clock;
unsigned long dt_gyro;
unsigned long tc_gyro;
unsigned long dtc_gyro;

unsigned long t_dmp;   // task dmp mpu
unsigned long t_dmp_p = 10000; // task dmp mpu period
unsigned long tp_dmp_clock;
unsigned long dt_dmp;
unsigned long tc_dmp;
unsigned long dtc_dmp;

// ================================================================
// ===                 FUNCTIONS DEFINITIONS                    ===
// ================================================================

void error_led();

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock.
  Wire.setWireTimeout(1000, true);

  pinMode(LED_PIN, OUTPUT);

#ifdef DEBUG
  Serial.begin(115200);
  while (!Serial);

  //  Serial.println(F("Initializing I2C devices..."));
#endif
  mpu.initialize();
  pinMode(MPU_INT_PIN, INPUT);

#ifdef DEBUG
  // verify connection
  //  Serial.println(F("Testing device connections..."));
  //  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
#else
  if (!mpu.testConnection()) error();
#endif

  // load and configure the DMP
  //  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS);
  delay(10);
  if (!mpu.getFullScaleGyroRange() == MPU6050_GYRO_FS) error();

  // supply your own gyro offsets here, scaled for min sensitivity
  //  mpu.setXGyroOffset(51);
  //  mpu.setYGyroOffset(8);
  //  mpu.setZGyroOffset(21);
  //  mpu.setXAccelOffset(1150);
  //  mpu.setYAccelOffset(-50);
  //  mpu.setZAccelOffset(1060);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    //    Serial.println();
    //    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready

    //    mpu.setRate(0x27); // 100Hz aprox
    //    mpu.setDLPFMode(MPU6050_DLPF_BW_256);

    mpu.setRate(0x04); // 100Hz aprox
    mpu.setDLPFMode(MPU6050_DLPF_BW_98);

    //    mpu.setDLPFMode(MPU6050_DLPF_BW_256);
    //    mpu.setDLPFMode(MPU6050_DLPF_BW_188);
    //    mpu.setDLPFMode(MPU6050_DLPF_BW_98);
    //    mpu.setDLPFMode(MPU6050_DLPF_BW_42);
    //    mpu.setDLPFMode(MPU6050_DLPF_BW_20);
    //    mpu.setDLPFMode(MPU6050_DLPF_BW_10);
    //    mpu.setDLPFMode(MPU6050_DLPF_BW_5);

    //    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    //    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    //    Serial.print(digitalPinToInterrupt(MPU_INT_PIN));
    //    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  //  g_scale = mpu.getFullScaleGyroRange();
  //  if(g_scale == MPU6050_GYRO_FS_250) Serial.println("MPU6050_GYRO_FS_250");
  //  if(g_scale == MPU6050_GYRO_FS_500) Serial.println("MPU6050_GYRO_FS_500");
  //  if(g_scale == MPU6050_GYRO_FS_1000) Serial.println("MPU6050_GYRO_FS_1000");
  //  if(g_scale == MPU6050_GYRO_FS_2000) Serial.println("MPU6050_GYRO_FS_2000");
  //  while(1);

  t_clock = micros();
  t_gyro = t_clock + t_gyro_p;
  t_dmp = t_clock + t_dmp_p;
}

void loop() {
  t_clock = micros();

    if(t_clock >= t_gyro){

      tc_gyro = micros();  
      mpu.getRotation(&gx, &gy, &gz);
      gxyz[0] = gx * MPU6050_G_GAIN;
      gxyz[1] = gy * MPU6050_G_GAIN;
      gxyz[2] = gz * MPU6050_G_GAIN;
    #ifdef OUTPUT_READABLE_GYRO
      Serial.print(gxyz[0]); Serial.print(",");
      Serial.print(gxyz[1]); Serial.print(",");
      Serial.print(gxyz[2]); Serial.print(",");
    #endif
      dtc_gyro = micros() - tc_gyro;
    
      t_gyro += t_gyro_p;
      dt_gyro = t_clock - tp_gyro_clock;
      tp_gyro_clock = t_clock;
      Serial.print(t_clock);
      Serial.print(",");
//      Serial.print(tc_gyro);
//      Serial.println();
  }

  if (t_clock >= t_dmp) {

    tc_dmp = micros();  
    mpu_data();
    dtc_dmp = micros() - tc_dmp;

    t_dmp += t_dmp_p;
    dt_dmp = t_clock - tp_dmp_clock;
    tp_dmp_clock = t_clock;
//    Serial.println(dtc_dmp);
//      Serial.print(tc_dmp);
//      Serial.println();
  }
}

void error() {
  while (1) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}


void mpu_data() {
  // if programming failed, don't try to do anything
  //  if (!dmpReady || !mpu.dmpPacketAvailable()) { delayMicroseconds(500); return; }
  int count = 1;
  while (!dmpReady && count <= 2) {
    count++;
    delayMicroseconds(500);
  }
  if(!dmpReady && !mpu.dmpPacketAvailable()) return;
  
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet

    Serial.println(1);
      
#ifdef OUTPUT_READABLE_EULER
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    //    Serial.print("euler\t");
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[2] * 180 / M_PI);
    Serial.print("\t");
    Serial.println();
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[2] * 180 / M_PI);
    Serial.print("\t");
    //    Serial.println();
#endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}
