#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define MIC A0

unsigned long LONG_MAX = 4294967295;

//threshold values to test against
int32_t IDLE_ACCEL = 4000 // TODO figure out this value: equal to gravity

int32_t lowAcceleration = 4096;
int32_t excitedAcceleration = 12000;
int32_t angryAcceleration = 22000;
int32_t lowVolume = 128;
int32_t excitedVolume = 256;
int32_t angryVolume = 768;

//Tracking variables
byte idle = 0;
byte happy = 1;
byte sad = 2;
byte excited = 3;
byte angry = 4;

byte prevState = 0;
byte state = 0;
bool canAct = true;
unsigned long actionStart = LONG_MAX;

boolean active = false;
unsigned long timeSinceIdle = LONG_MAX;

//action states
byte idling = 0;
byte starting = 1;
byte executing = 2;
byte finishing = 3;
byte actionState = 0;

//Servo Creation
Servo extenderServo;
Servo armBaseServo;
Servo armUpperServo;

//Accelerometer Creation
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup() {
  Serial.begin(9600);
  Serial.println("setup()");
  
  //Initialize all servos
  Serial.println("initializing servos...");
  extenderServo.attach(9);
  armBaseServo.attach(10);
  armUpperServo.attach(11);

  //Create Accelerometer Communication
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.println("initializing I2C...");
  accelgyro.initialize();

  Serial.println("testing MPU6050 device connection...");
  Serial.println(accelgyro.testConnection() ? "success" : "failure");
}

void setState(s) {
  prevState = state;
  state = s;
  canAct = false;
  actionState = starting;
}

void doAction() {
  switch (actionState) {
    case idling:
      idle();
      break;

    case starting:
      start();
      actionState = executing;
      break;

    case executing:
      execute();
      break;

    case finishing:
      finish();
      actionState = idling;
      canAct = True;
      break;
  }
}

void idle() {
}

void start() {
  actionStart = millis();
}

void execute() {
  unsigned long t = millis() - actionStart;
  switch(state) {
    case excited:
      if (t < 200) {
        extenderServo.write(180);
      } else if (t < 400) {
        extenderServo.write(0);
      } else if (t < 600) {
        extenderServo.write(180);
      } else if (t < 800) {
        extenderServo.write(0);
      } else {
        extenderServo.write(90);
        actionState = finishing;
      }
      break;

    case happy:
      if (t < 200) {
        armBaseServo.write(110);
      } else if (t < 600) {
        armUpperServo.write(110);
      } else if (t < 1000) {
        armUpperServo.write(70);
      } else {
        armBaseServo.write(90);
        armUpperServo.write(90);
        actionState = finishing;
      }
      break;

    case angry:
      if (t < 200) {
        armUpperServo.write(180);
      } else if (t < 400) {
        armUpperServo.write(0);
      } else if (t < 600) {
        armUpperServo.write(180);
      } else if (t < 800) {
        armUpperServo.write(0);
      } else {
        armUpperServo.write(90);
        actionState = finishing;
      }
      break;

    case sad:
      if (t < 500) {
        extenderServo.write(70);
      } else if (t < 1000) {
        extenderServo.write(110);
      } else {
        extenderServo.write(90);
        actionState = finishing;
      }
  }
}

void finish() {
  actionStart = LONG_MAX;
}

void loop() {    
  
  //Check each sensor's value
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  int gain = analogRead(MIC); // range 0-1023

  int32_t accel_mag = abs(ax) + abs(ay) + abs(az) - IDLE_ACCEL;

  unsigned long currentTime = millis();
  
  //Check sensor values against the thresholds
  //If a sensor is within a threshold, do the action associated with said threshold.
  if (active) {
    timeSinceIdle = currentTime;
  } else if (timeSinceIdle + 10000 > currentTime && canAct) {
    setState(sad);
  } else if (timeSinceIdle + 10000 > currentTime && canAct && state == sad) {
    prevState = state;
    state = idle;
  }

  if (canAct) {
    if (gain >= angryVolume) {
      setState(angry);
    } else if (gain >= excitedVolume) {
      setState(excited);
    } else if (gain >= lowVolume) {
      if (state == happy || state == excited || state == sad) {
        setState(happy);
      } else {
        setState(sad);
      }
    }
  }

  if (canAct){
    if (accel_mag >= angryAcceleration) {
      setState(angry);
    } else if (accel_mag >= excitedAcceleration) {
      setState(excited);
    } else if (accel_mag >= lowAcceleration) {
      if (state == happy || state == excited || state == sad) {
        setState(happy);
      } else {
        setState(sad);
      }
    }
  }

  doAction();
}

