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
int32_t speech_high = 60;
int32_t speech_low = 8;
int32_t toss_low = 3000;
int32_t toss_high = 15000;
int32_t hug_low = 700;
int32_t hug_high = 900;


//speech filter vars:
float speech_alpha = 0.55; 
float speech_past=0;
float speech_updated,filtered_mic;
float hug_alpha = 0.1; 
float hug_past=0;
float hug_updated;
float force_alpha = 0.85; 
float force_past=0;
float force_updated;
float time_since_behavior = 0;
float LONG_HUG_TIME = 5000;

//input detection variables
int speech_state = 0; //0 = no speech, 1 = whisper, 2 = talking, 3 = shouting 
unsigned long speech_start_time =0; 
int force_state = 0; //0 = no acceleration, 1 = toss, 2 = throw, 3 = slam
unsigned long force_start_time = 0;
int hug_state = 0; //0 = no hug, 1 = short hug, 2 = long hugno
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
Servo servo1;
Servo servo2;
Servo servo3;

//LEDS creation
Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, 6, NEO_GRB + NEO_KHZ800);

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
  servo1.attach(9);
  servo2.attach(10);
  servo3.attach(11);

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
  analogWrite(vibpin, 0);
  strip.begin();
  strip.show();
  push(true);
  push(false);
  
}

float speech_lowpass_step(float input){
       speech_updated = input*(1-speech_alpha) + speech_past*(speech_alpha);
       speech_past = speech_updated;
       return speech_updated;
}
float hug_lowpass_step(float input){s
       hug_updated = input*(1-hug_alpha) + hug_past*(hug_alpha);
       hug_past = hug_updated;
       return hug_updated;
}
float force_lowpass_step(float input){
       force_updated = input*(1-force_alpha) + force_past*(force_alpha);
       force_past = force_updated;
       return force_updated;
}

int detect_speech(int reading){
  
  filtered_mic = abs(speech_lowpass_step(reading));
  
  debugPrint("filtered_mic: ");
  debugPrintln(String(filtered_mic));
  
  if ((speech_high <= filtered_mic && filtered_mic < speech_high+50) && (speech_state != 3) && millis() - speech_start_time > 100){
    speech_state = 3;
    speech_start_time = millis();
    debugPrintln("shouts detected");
    return 3;
  } else if ((speech_low+10 <= filtered_mic && filtered_mic < speech_high) && (speech_state!= 2)&& millis() - speech_start_time > 100){
    speech_state = 0;
    speech_start_time = millis();
    debugPrintln("speech detected");
    return 2;
  } else if ((speech_low <= filtered_mic && filtered_mic < speech_low+10) && (speech_state!= 2)&& millis() - speech_start_time > 100){
    speech_state = 1;
    speech_start_time = millis();
    debugPrintln("whisper detected");
    return 1;
  } else if (filtered_mic < speech_low && speech_state != 0 && millis() - speech_start_time > 100){
    speech_state = 0;
    debugPrintln("end of speech detected");
    return 0;
  }
  debugPrint("no speech change, current state is: ");
  debugPrintln(String(speech_state));
  return 0;
}

int detect_hug(int reading){
  reading = hug_lowpass_filter(abs(reading));
  if (hug_low < reading && reading < hug_high && hug_state == 0 && millis() - hug_start_time > 10 ){
    hug_state = 1;
    hug_start_time = millis();
    debugPrintln("hug detected");
    return 1;
  } else if (hug_low < reading && reading < hug_high && millis() -hug_start_time >= LONG_HUG_TIME ){
    hug_state = 2;
    debugPrintln(" long hug detected");
    return 2;
  } else if (reading < hug_low && millis() - hug_start_time > 10 ){
    hug_state = 0;
    debugPrintln("end of hug" && millis() - hug_start_time > 10 );
    return 0;
  }
  
}

int detect_force(int reading) {
  reading = force_lowpass_filter(abs(reading));
  if ((toss_low-25 <= reading && reading < toss_high+50) && (force_state != 1) && millis() - force_start_time > 200){
    force_state = 1;
    force_start_time = millis();    
    debugPrintln("toss detected");
    return 1;
  } else if ((20000 <= reading) && (force_state!= 3) && millis() - force_start_time > 200) {
      force_state = 3;
      force_start_time = millis();    
      debugPrintln("slam detected");
      return 3;
  } else if ((toss_high+1000 <= reading) && (force_state!= 2) && millis() - force_start_time > 200){
      force_state = 2;
      force_start_time = millis();    
      debugPrintln("throw detected");
      return 2;
  } else if (abs(filtered_mic) <= speech_low && speech_state != 0 && millis() - force_start_time > 20){
      force_state = 0;
      debugPrintln("end of toss detected");
      return 0;
  }
  debugPrint("no toss change, current state is: ");
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
      setLights(1,1.0); 
      if (t < 200) {
        vibrate_on = true;
        servo1.write(100);
      } else if (t < 400) {
        setLights(1,1.0); 
        vibrate_on = false;
        servo1.write(80);
      } else if (t < 600) {
        setLights(1,1.0); 
        vibrate_on = true;
        servo1.write(100);
      } else if (t < 800) {
        setLights(1,1.0); 
        vibrate_on = false;
        servo1.write(80);
      } else {
        setLights(1,1.0); 
        servo1.write(90);
        actionState = finishing;
      }
      break;

    case happy:
      setLights(2,0.7);
      if (t < 200) {
        vibrate_on = true; 
        servo2.write(110);
      } else if (t < 600) {
        setLights(2,0.7);
        vibrate_on = false;
        servo3.write(100);
      } else if (t < 1000) {
        setLights(2,0.7);
        servo3.write(80);
      } else {
        setLights(2,0.7);
        servo2.write(90);
        servo3.write(90);
        actionState = finishing;
      }
      break;

    case angry:
    setLights(0,0.9);
      if (t < 200) {
        vibrate_on = true;
        servo3.write(110);
      } else if (t < 400) {
        setLights(0,0.9);
        servo3.write(70);
      } else if (t < 600) {
        setLights(0,0.9);
        servo3.write(110);
      } else if (t < 800) {
        setLights(0,0.9);
        servo3.write(70);
      } else {
        setLights(0,0.9);
        vibrate_on = false;
        servo3.write(90);
        actionState = finishing;
      }
      break;

    case sad:
      setLights(4,0.3);
      if (t < 500) {
        setLights(4,0.3);
        vibrate_on = true;
        servo1.write(80);
        vibrate_on = false;
      } else if (t < 1000) {
        setLights(4,0.3);
        servo1.write(100);
      } else {
        setLights(4,0.3);
        servo1.write(90);
        actionState = finishing;
      }
    
    case wake:
      setLights(5,1.0); 
      setState(excited);
      break;
  }
}

void finish() {
  actionStart = LONG_MAX;
}

void setLights(int color, float brightness){
  int red, blue, green; 
  switch(color){
    case 0:
     //RED
     red = 255;
     green =0;
     blue = 0;
    case 1:
     //ORANGE
     red = 255;
     green = 153;
     blue = 51;
    case 2:
    //YELLOW
    red = 255;
    green = 255;
    blue = 0;
    case 3:
    //GREEN
    red = 0;
    green = 220;
    blue = 20;
    case 4:
    //BLUE
    red = 51;
    green = 51;
    blue = 255;
    case 5:
    //WHITE
    red = 255;
    green = 255;
    blue = 255;
  }
  float prob = brightness *100;
  for (i = 0; i < 30; i++){
    if (rand() %100 <= prob){
      strip.setPixelColor(i, red,blue, green);
      }
    }
  }
  strip.show();


void set_servos(int servo_1_val, int servo_2_val, int servo_3_val){
  servo_1.write(servo_1_val);
  servo_2.write(servo_2_val);
  servo_3.write(servo_3_val);
  }

void wobble(){
    current = millis() % 500;
    if (current %2 == 0){
    reading_1 = servo_1.read();
    reading_2 = servo_2.read();
    reading_3 = servo_3.read();
      if (current < 167){
        servo_1.write(reading_1+1);
        servo_2.write(reading_2-1);
        servo_3.write(reading_3-1);
        }
  
      else if (current < 334){
        servo_1.write(reading_1-1);
        servo_2.write(reading_2+1);
        servo_3.write(reading_3-1);
        }
      else{
        servo_1.write(reading_1-1);
        servo_2.write(reading_2-1);
        servo_3.write(reading_3+1);
        }
    }
  }


void push(bool up){
  if (up){
    servo_1.write(90);
    servo_2.write(90);
    servo_3.write(90);
    }
  else{
    servo_1.write(0);
    servo_2.write(0);
    servo_3.write(0);
    }
}

void loop() {
  
  //Check each sensor's value
  wobble();
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  int gain = abs(analogRead(MIC)-IDLE_GAIN); // range 0-1023
  detect_speech(gain);
  
  int32_t accel_mag = pow(pow(ax,2)+pow(ay,2)+pow(az,2), .5)-IDLE_ACCEL;

  int force = analogRead(FSR);
  detect_force(accel_mag);
  unsigned long currentTime = millis();

//  debugPrint("accel_mag = ");
//  debugPrintln(String(accel_mag));
//  debugPrint(", gain = ");
//  debugPrint(String(gain));
//  debugPrint(", force = ");
//  debugPrintln(String(force));

//  return;

//  if (canAct)
//    setState(excited);
//  doAction();
//  return;
  
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
  if (hug_state == 1 && asleep){
//    setState(wake);
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

  if(vibrate_on) {
    analogWrite(vibpin, 153);
  } else {
    analogWrite(vibpin, 0);
  }

  printCount++;
  doAction();
}

