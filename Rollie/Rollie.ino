#include <Servo.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Adafruit_NeoPixel.h>


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define MIC A0
#define FSR A1

unsigned long LONG_MAX = 4294967295;

int vibpin = 6;

//threshold values to test against
int32_t IDLE_ACCEL = 16000; // TODO figure out this value: equal to gravity
int32_t IDLE_GAIN = 250;

int32_t lowAcceleration = 4096;
int32_t excitedAcceleration = 12000;
int32_t angryAcceleration = 16000;
int32_t lowVolume = 40;
int32_t excitedVolume = 80;
int32_t angryVolume = 160;
int32_t speech_high = 50;
int32_t speech_low = 35;
int32_t toss_low = 4096;
int32_t toss_high = 15000;
int32_t hug_low = 700;
int32_t hug_high = 900;


//filter vars:
float speech_alpha = 0.25; 
float speech_past=0;
float speech_updated,filtered_mic;
float hug_alpha = 0.1; 
float hug_past=0;
float hug_updated;
float toss_alpha = 0.9; 
float toss_past=0;
float toss_updated;
float time_since_behavior = 0;
float LONG_HUG_TIME = 5000;

//input detection variables
int speech_state = 0; // 0 = no speech, 1 = whisper, 2 = talking, 3 = shouting 
unsigned long speech_start_time =0; 
int toss_state = 0; // 0 = no acceleration, 1 = toss, 2 = throw, 3 = slam
unsigned long force_start_time = 0;
int hug_state = 0; // 0 = no hug, 1 = short hug, 2 = long hugno
unsigned long hug_start_time = 0; 
unsigned long vibrate_start_time = 0;
bool vibrate_on = false;


//Tracking variables
const byte idle = 0;
const byte happy = 1;
const byte sad = 2;
const byte excited = 3;
const byte angry = 4;
const byte wake = 5; 

byte prevState = 0;
byte state = 0;
bool asleep = true;
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

//leds
Adafruit_NeoPixel LEDs = Adafruit_NeoPixel(60, 6, NEO_GRB + NEO_KHZ800);


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
  analogWrite(vibpin, 153);
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
  analogWrite(VIBPIN, 0);
>>>>>>> 509fe5b4ff7ac512038c38bb47448aaef36998e0
}

float speech_lowpass_step(float input){
       speech_updated = input*(1-speech_alpha) + speech_past*(speech_alpha);
       speech_past = speech_updated;
       return speech_updated;
}
float hug_lowpass_step(float input){
       hug_updated = input*(1-hug_alpha) + hug_past*(hug_alpha);
       hug_past = hug_updated;
       return speech_updated;
}
float toss_lowpass_step(float input){
       toss_updated = input*(1-toss_alpha) + toss_past*(toss_alpha);
       toss_past = toss_updated;
       return toss_updated;
}

int detect_speech(int reading){
  
  filtered_mic = abs(speech_lowpass_step(reading));
  
//  debugPrint("filtered_mic: ");
//  debugPrintln(String(filtered_mic));
  
  if ((speech_high <= filtered_mic && filtered_mic < speech_high+50) && (speech_state != 3)){
    speech_state = 3;
    debugPrintln("shouts detected");
    return 3;
  } else if ((speech_low+10 <= filtered_mic && filtered_mic < speech_high) && (speech_state!= 2)){
    speech_state = 0;
    debugPrintln("speech detected");
    return 2;
  } else if ((speech_low <= filtered_mic && filtered_mic < speech_low+10) && (speech_state!= 2)){
    speech_state = 1;
    debugPrintln("whisper detected");
    return 1;
  } else if (filtered_mic < speech_low && speech_state != 0){
    speech_state = 0;
    debugPrintln("end of speech detected");
    return 0;
  }
//  debugPrint("no speech change, current state is: ");
//  debugPrintln(String(speech_state));
  return 0;
}
int detect_hug(int reading){

  reading = hug_lowpass_step(abs(reading));
  if (hug_low < reading && reading < hug_high && hug_state == 0){
    hug_state = 1;
    hug_start_time = millis();
    debugPrintln("hug detected");
    return 1;
  } else if (hug_low < reading && reading < hug_high && millis() -hug_start_time >= LONG_HUG_TIME){
    hug_state = 2;
    debugPrintln(" long hug detected");
    return 2;
  } else if (reading < hug_low && hug_state != 0){
    hug_state = 0;
    debugPrintln("end of hug");
    return 0;
    }
  }


int detect_accel(int reading) {
  reading = toss_lowpass_step(abs(reading));
  if ((toss_low-25 <= reading && reading < toss_high+50) && (toss_state != 1)){
    toss_state = 1;
    debugPrintln("toss detected");
    return 1;
  } else if ((20000 <= reading) && (toss_state!= 3)) {
    toss_state = 3;
    debugPrintln("slam detected");
    return 3;
  } else if ((toss_high+1000 <= reading) && (toss_state!= 2)){
    toss_state = 2;
    debugPrintln("throw detected");
    return 2;
  } else if (abs(filtered_mic) <= speech_low && toss_state != 0){
    toss_state = 0;
    debugPrintln("end of toss detected");
    return 0;
  }
//  debugPrint("no toss change, current state is: ");
//  debugPrintln(String(toss_state));
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
        vibrate_on = true;
        extenderServo.write(100);
      } else if (t < 400) {
        
        vibrate_on = false;
        extenderServo.write(80);
      } else if (t < 600) {
        
        vibrate_on = true;
        extenderServo.write(100);
      } else if (t < 800) {
        
        vibrate_on = false;
        extenderServo.write(80);
      } else {
        extenderServo.write(90);
        actionState = finishing;
      }
      break;

    case happy:
      if (t < 200) {
        vibrate_on = true; 
        armBaseServo.write(110);
      } else if (t < 600) {
        vibrate_on = false;
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
        vibrate_on = true;
        armUpperServo.write(110);
      } else if (t < 400) {
        armUpperServo.write(70);
      } else if (t < 600) {
        armUpperServo.write(110);
      } else if (t < 800) {
        armUpperServo.write(70);
      } else {
        vibrate_on = false;
        armUpperServo.write(90);
        actionState = finishing;
      }
      break;

    case sad:
      if (t < 500) {
        vibrate_on = true;
        extenderServo.write(80);
        vibrate_on = false;
      } else if (t < 1000) {
        extenderServo.write(100);
      } else {
        extenderServo.write(90);
        actionState = finishing;
      }
    case wake:
      setState(excited);
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
  
  int32_t accel_mag = pow(pow(ax,2)+pow(ay,2)+pow(az,2), .5)-IDLE_ACCEL;

  int force = analogRead(FSR);
  detect_hug(force);
  detect_accel(accel_mag);
  unsigned long currentTime = millis();

//  debugPrint("accel_mag = ");
//  debugPrint(String(accel_mag));
//  debugPrint(",\tgain = ");
//  debugPrint(String(gain));
//  debugPrint(",\tforce = ");
//  debugPrint(String(force));
//  debugPrint(",\tvib = ");
//  debugPrintln(String(vibrate_on));

  if (canAct && hug_state == 1) {
    setState(excited);
  }

  return;

//  if (canAct)
//    setState(excited);
//  doAction();
//  return;
  
  //Check sensor values against the thresholds
  //If a sensor is within a threshold, do the action associated with said threshold.

//  if (active) {
//    timeSinceIdle = currentTime;
//  } else if (timeSinceIdle + 10000 > currentTime && canAct) {
//    setState(sad);
//  } else if (timeSinceIdle + 10000 > currentTime && canAct && state == sad) {
//    prevState = state;
//    state = idle;
//  }
//  if (toss_state !=0 || speech_state != 0){
//    time_since_behavior = millis();
//  } else {
//    time_since_behavior = 0;
//  }

//  debugPrint("hug_state = ");
//  debugPrint(String(hug_state));
//  debugPrint(", toss_state = ");
//  debugPrint(String(toss_state));
//  debugPrint(", speech_state = ");
//  debugPrintln(String(speech_state));

//  if (canAct) {
//    if (hug_state == 1 && asleep) {
////      setState(wake);
//      asleep = false;
//    } else if (millis() - time_since_behavior > 15000 || hug_state == 2) {
//      setState(idle);
//      asleep = true;
//    } else if (toss_state == 3) {
//      setState(angry);
//    } else if (toss_state == 1 || speech_state == 2) {
//      setState(happy);
//    } else if (toss_state == 2 || speech_state == 3) {
//      setState(sad);
//    } else if (speech_state == 1 || hug_state == 1) {
//      setState(happy);
//    }
//  }

  if(vibrate_on) {
    analogWrite(VIBPIN, 153);
  } else {
    time_since_behavior = 0;
  }
  if (hug_state == 1 && asleep){
    setState(wake);
    asleep = false;
    }
  else if (millis() - time_since_behavior > 15000 || hug_state == 2){
    setState(idle);
    asleep = true;
  } else if (force_state == 3) {
    setState(angry);
  } else if (force_state == 1 || speech_state == 2) {
    setState(happy);
  } else if (force_state == 2 || speech_state == 3) {
    setState(sad);
  } else if (speech_state == 1 || hug_state == 1) {
    setState(happy);
  }

  if(vibrate_on){
    analogWrite(vibpin, 153);
    }
  else{
    analogWrite(vibpin, 0);
    }

  printCount+=0;
  doAction();
}

