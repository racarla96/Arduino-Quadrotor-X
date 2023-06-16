#define DEBUG

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
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
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


unsigned long t_k_loop;
unsigned long t_k_1_loop;
unsigned long dt_loop;

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
//  Wire.setWireTimeout(1000, true);

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
  if (!mpu.testConnection()) {
    while (1) {
      error_led();
    }
  }
#endif

  // load and configure the DMP
//  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

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

    mpu.setRate(0x00); // 100Hz aprox
    mpu.setDLPFMode(MPU6050_DLPF_BW_188);
    
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

  t_k_loop = micros();
  t_k_1_loop = t_k_loop;
}

void loop() {
  // if programming failed, don't try to do anything
//  if (!dmpReady || !mpu.dmpPacketAvailable()) { delayMicroseconds(500); return; }
  if (!dmpReady) { return; }
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

#ifdef OUTPUT_READABLE_EULER
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
//    Serial.print("euler\t");
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180 / M_PI);
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
    /*
      mpu.dmpGetAccel(&aa, fifoBuffer);
      Serial.print("\tRaw Accl XYZ\t");
      Serial.print(aa.x);
      Serial.print("\t");
      Serial.print(aa.y);
      Serial.print("\t");
      Serial.print(aa.z);
      mpu.dmpGetGyro(&gy, fifoBuffer);
      Serial.print("\tRaw Gyro XYZ\t");
      Serial.print(gy.x);
      Serial.print("\t");
      Serial.print(gy.y);
      Serial.print("\t");
      Serial.print(gy.z);
    */
    Serial.println();
#endif

    t_k_loop = micros();
    dt_loop = t_k_loop - t_k_1_loop;
    t_k_1_loop = t_k_loop;
    Serial.println(dt_loop);

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}

void error_led() {
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(100);
}
