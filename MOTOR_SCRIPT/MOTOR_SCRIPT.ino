// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// Defining pin LM2899 switch control
int switch_13 = 5;
int switch_24 = 6;


#define OUTPUT_READABLE_YAWPITCHROLL

// Unccomment if you are using an Arduino-Style Board
#define ARDUINO_BOARD

// Uncomment if you are using a Galileo Gen1 / 2 Board
// #define GALILEO_BOARD

#define LED_PIN 13  // (Galileo/Arduino is 13)
bool blinkState = false;

// PID parameters
float Kp(0), Ki(0), Kd(0);
float xError(0), xCumulativeError(0), xRateError(0), xLastError(90);

// Reference angle
#define thetaX_ref 0
#define thetaU_ref 0

// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
VectorFloat gravity;    // [x, y, z]            gravity vector
Quaternion q;           // [w, x, y, z]         quaternion container
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float xy[2];           // [x,  y]              x/y/z axis container
float xy_old[2] {0};  // [x,  y]              x/y/z axis container of previous reading



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

// This function is not required when using the Galileo
volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                    GYROSCOPE ROUTINE                     ===
// ================================================================

void gyro_initialize() {
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  while (!Serial)
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(F("MPU6050 connection "));
  Serial.print(mpu.testConnection() ? F("successful") : F("failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read())
    ;  // empty buffer

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(43);
  mpu.setYGyroOffset(16);
  mpu.setZGyroOffset(18);
  mpu.setXAccelOffset(-443);   // 1688 factory default for my test chip
  mpu.setYAccelOffset(-1969);  // 1688 factory default for my test chip
  mpu.setZAccelOffset(1064);   // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
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

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

void gyro_read() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available

    #ifdef ARDUINO_BOARD
      delay(10);
    #endif

    #ifdef GALILEO_BOARD
      delay(10);
    #endif

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = true;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    float e = 90;
    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      xy[0] = ypr[2] * 180 / M_PI;
      xy[1] = ypr[1] * 180 / M_PI;
      Serial.print("xyz\t");
      Serial.print(xy[0]);
      Serial.print(" - ");
      Serial.print(xy_old[0]);
      Serial.print("\t");
      Serial.print(xy[1]);
      Serial.print(" - ");
      Serial.println(xy_old[1]);

        e = -xy[0];
    #endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}


// ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
// ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
// ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

// pin1 - forward control pin
// pin2 - backward control pin
void spinForward(float e) {

  e *= 10;
  int PWM;

  PWM = map(e, 0, 900, 0, 255);

  analogWrite(switch_24, 0);
  analogWrite(switch_13, PWM);
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  Serial.begin(115200);

  pinMode(switch_13, OUTPUT);
  pinMode(switch_24, OUTPUT);

  gyro_initialize();  // gyroscope initialization
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  gyro_read();

  // Proportional X error
  xError = xy[0] - thetaX_ref;
  // Cumulative X error
  xCumulativeError += xError * .001f;
  // Rate X error
  xRateError = (xError - xLastError) / .001f;


  xy_old[0] = xy[0];
  xy_old[1] = xy[1];
}
