#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


int accelerometerXPin = A0;
int accelerometerYPin = A1;
int accelerometerZPin = A2;
int microphonePin = A3;

//threshold values to test against
int lowAcceleration = 10;
int excitedAcceleration = 30;
int angryAcceleration = 50;
int lowVolume = 10;
int excitedVolume = 30;
int angryVolume = 50;

//Tracking variables
char oldState = "";
char state = "";
bool canAct = true;
int countSinceInteraction = 0;
int aloneUntilSad = 100;

//Servo Creation
Servo armBaseServo;
Servo armUpperServo;
Servo extenderServo;

//Accelerometer Creation
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int netAcceleration = 0;

void setup() {
    Serial.begin(9600); // Begin the Comms with Arduino
    
    //Initialize all servos
    armBaseServo.attach(0);
    armUpperServo.attach(1);
    extenderServo.attach(2);

    //Create Accelerometer Communication
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    accelgyro.initialize();


}

void loop() {    
  state = "";
  
  //Check each sensor's value
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  int microphoneVolume = analogRead(microphonePin);

  netAcceleration = (ax^2 + ay^2 + az^2)^1/2;
  
  //Check sensor values against the thresholds
  //If a sensor is within a threshold, do the action associated with said threshold.
  if (countSinceInteraction > aloneUntilSad) {
    state = "sad";
  };
  
  if (microphoneVolume >= angryVolume) {
    state = "angry";
  } else if (microphoneVolume >= excitedVolume) {
    state = "excited";
  } else if (microphoneVolume >= lowVolume) {
    if (oldState == "happy" || oldState == "excited") {
      state = "happy";
    } else {
      state = "sad";
    };
  };

  if (netAcceleration >= angryAcceleration) {
    state = "angry";
  } else if (netAcceleration >= excitedAcceleration) {
    state = "excited";
  } else if (netAcceleration >= lowAcceleration) {
    if (oldState == "happy" || oldState == "excited") {
      state = "happy";
    } else {
      state = "sad";
    };
  };

  //check the current state and run an action dependent on that state
  if (canAct == true) {
    if (state == "excited") {
      canAct = false;
      
      armUpperServo.write(50);
      delay(50);
      armUpperServo.write(90);
      armBaseServo.write(30);
      delay(2000);
      armBaseServo.write(90);
      armUpperServo.write(140);
      delay(50);
      armUpperServo.write(90);
      
      canAct = true;
    } else if (state == "happy") {
      canAct = false;

      for (int i=1;i<=6;i++) {
        armUpperServo.write(60);
        extenderServo.write(70);
        delay(500);
        extenderServo.write(110);
        armUpperServo.write(130);
        delay(500);
        extenderServo.write(90);
        armUpperServo.write(90);
      }
      
      canAct = true;
    } else if (state = "sad") {
      canAct = false;

      armUpperServo.write(80);
      delay(500);
      armUpperServo.write(90);
      armBaseServo.write(85);
      delay(5000);
      armBaseServo.write(90);
      armUpperServo.write(100);
      delay(500);
      armUpperServo.write(90);
      
      canAct = true;
    } else if (state = "angry") {
      canAct = false;

      armUpperServo.write(20);
      extenderServo.write(40);
      delay(200);
      armUpperServo.write(90);
      extenderServo.write(90);
      delay(1000);
      armUpperServo.write(120);
      delay(300);
      armUpperServo.write(30);
      delay(400);
      armUpperServo.write(140);
      delay(200);
      armUpperServo.write(25);
      delay(160);
      armUpperServo.write(90);
      extenderServo.write(140);
      delay(200);
      extenderServo.write(90);
      
      canAct = true;
    } else {
      countSinceInteraction +=1;
      delay(100);
    };
  };
  oldState = state;
}

