#include <Servo.h>
#include <I2Cdev.h>
#include <MPU6050.h>


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define MIC A0
#define FSR A1

unsigned long LONG_MAX = 4294967295;

//threshold values to test against
int32_t IDLE_ACCEL = 16000; // TODO figure out this value: equal to gravity
int32_t IDLE_GAIN = 250;

int32_t lowAcceleration = 4096;
int32_t excitedAcceleration = 12000;
int32_t angryAcceleration = 16000;
int32_t lowVolume = 40;
int32_t excitedVolume = 80;
int32_t angryVolume = 160;
int32_t speech_high = 120;
int32_t speech_low = 35;
int32_t toss_low = 1350;
int32_t toss_high = 12000;

//speech filter vars:
float speech_alpha = 0.95; 
float speech_past=0;
float speech_updated,filtered_mic;
float time_since_behavior = 0;

//input detection variables
int speech_state = 0; //0 = no speech, 1 = whisper, 2 = talking, 3 = shouting 
unsigned long speech_start_time =0; 
int force_state = 0; //0 = no acceleration, 1 = toss, 2 = throw, 3 = slam
unsigned long force_start_time = 0;
bool hug_state = false;
unsigned long hug_start_time = 0; 


//Tracking variables
const byte idle = 0;
const byte happy = 1;
const byte sad = 2;
const byte excited = 3;
const byte angry = 4;

byte prevState = 0;
byte state = 0;
bool canAct = true;
unsigned long actionStart = LONG_MAX;

boolean active = false;
unsigned long timeSinceIdle = LONG_MAX;

//action states
const byte idling = 0;
const byte starting = 1;
const byte executing = 2;
const byte finishing = 3;
byte actionState = 0;

//Servo Creation
Servo extenderServo;
Servo armBaseServo;
Servo armUpperServo;

//Accelerometer Creation
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

uint16_t printCount = 0;

void debugPrint(String msg) {
  if (printCount % 100 == 0) {
    Serial.print(msg);
  }
}

void debugPrintln(String msg) {
  if (printCount % 100 == 0) {
    Serial.println(msg);
  }
}

void setup() {
  Serial.begin(9600);
  debugPrintln("setup()");
  
  //Initialize all servos
  debugPrintln("initializing servos...");
  extenderServo.attach(9);
  armBaseServo.attach(10);
  armUpperServo.attach(11);

  //Create Accelerometer Communication
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  debugPrintln("initializing I2C...");
  accelgyro.initialize();

  debugPrintln("testing MPU6050 device connection...");
  debugPrintln(accelgyro.testConnection() ? "success" : "failure");
}

float lowpass_step(float input){
       speech_updated = input*(1-speech_alpha) + speech_past*(speech_alpha);
       speech_past = speech_updated;
       return speech_updated;
}

int detect_speech(int reading){
  
  filtered_mic = lowpass_step(reading);
  if ((speech_high <= abs(filtered_mic) <= speech_high+50)and(speech_state != 3)){
        speech_state = 3;
    debugPrintln("shouts detected");
    return 3;
    }
  else if ((speech_low+50<=abs(filtered_mic) <= speech_high) and (speech_state!= 2)){
    speech_state = 0;
    debugPrintln("speech detected");
    return 2;
    }
   else if ((speech_low<=abs(filtered_mic) <= speech_low+50) and (speech_state!= 2)){
    speech_state = 1;
    debugPrintln("whisper detected");
    return 1;
    }
  
  else if (abs(filtered_mic) <= speech_low && speech_state != 0){
    speech_state = 0;
    debugPrintln("end of speech detected");
    return 0;
    }
  debugPrint("no state change, current state is: ");
  debugPrintln(String(speech_state));
  return 0;
  }

int detect_force(int reading){
  if ((toss_low-25 <= abs(reading) <= toss_high+50)and(force_state != 1)){
        force_state = 1;
    debugPrintln("toss detected");
    return 1;
  } else if ((20000<=abs(reading)) and (force_state!= 3)) {
      force_state = 3;
      debugPrintln("slam detected");
      return 3;
  } else if ((toss_high+1000<=abs(reading)) and (force_state!= 2)){
      force_state = 2;
      debugPrintln("throw detected");
      return 2;
  } else if (abs(filtered_mic) <= speech_low && speech_state != 0){
      force_state = 0;
      debugPrintln("end of speech detected");
      return 0;
  }
  debugPrint("no state change, current state is: ");
  debugPrintln(String(force_state));
  return 0;
}


void setState(byte s) {
  prevState = state;
  state = s;
  canAct = false;
  actionState = starting;
}

void doAction() {
  switch (actionState) {
    case idling:
      laze();
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
      canAct = true;
      break;
  }
}

void laze() {
}

void start() {
  actionStart = millis();
}

void execute() {
  unsigned long t = millis() - actionStart;
  switch(state) {
    case excited:
      if (t < 200) {
        extenderServo.write(100);
      } else if (t < 400) {
        extenderServo.write(80);
      } else if (t < 600) {
        extenderServo.write(100);
      } else if (t < 800) {
        extenderServo.write(80);
      } else {
        extenderServo.write(90);
        actionState = finishing;
      }
      break;

    case happy:
      if (t < 200) {
        armBaseServo.write(110);
      } else if (t < 600) {
        armUpperServo.write(100);
      } else if (t < 1000) {
        armUpperServo.write(80);
      } else {
        armBaseServo.write(90);
        armUpperServo.write(90);
        actionState = finishing;
      }
      break;

    case angry:
      if (t < 200) {
        armUpperServo.write(110);
      } else if (t < 400) {
        armUpperServo.write(70);
      } else if (t < 600) {
        armUpperServo.write(110);
      } else if (t < 800) {
        armUpperServo.write(70);
      } else {
        armUpperServo.write(90);
        actionState = finishing;
      }
      break;

    case sad:
      if (t < 500) {
        extenderServo.write(80);
      } else if (t < 1000) {
        extenderServo.write(100);
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
  int gain = abs(analogRead(MIC)-IDLE_GAIN); // range 0-1023
  detect_speech(gain);
  
//  int gain = 0;

  int32_t accel_mag = pow(pow(ax,2)+pow(ay,2)+pow(az,2), .5)-IDLE_ACCEL;

  int force = analogRead(FSR);
  detect_force(force);
  unsigned long currentTime = millis();

//  debugPrint("accel_mag = ");
//  debugPrint(String(accel_mag));
//  debugPrint(", gain = ");
//  debugPrint(String(gain));
//  debugPrint(", force = ");
//  debugPrintln(String(force));

  if (canAct)
    setState(excited);
  doAction();
  return;
  
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
  if (force_state !=0 || speech_state != 0){
    time_since_behavior = millis();
  } else {
    time_since_behavior = 0;
  }

  if (millis() - time_since_behavior > 15000){
    setState(idle);
  } else if (force_state == 3) {
    setState(angry);
  } else if (force_state == 1 || speech_state == 2) {
    setState(happy);
  } else if (force_state == 2 || speech_state == 3) {
    setState(sad);
  } else if (speech_state == 1) {
    setState(happy);
  }

  printCount++;
  doAction();
}

