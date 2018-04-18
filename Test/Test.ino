#include <Servo.h>
#include <I2Cdev.h>
#include <MPU6050.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include <Wire.h>
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
  extenderServo.write(90);
  armBaseServo.attach(10);
  armBaseServo.write(90);
  armUpperServo.attach(11);
  armUpperServo.write(90);

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

void loop() {    
  
  //Check each sensor's value
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  int gain = abs(analogRead(MIC)-IDLE_GAIN); // range 0-1023
  int32_t accel_mag = pow(pow(ax,2)+pow(ay,2)+pow(az,2), .5)-IDLE_ACCEL;
  int force = analogRead(FSR);

  Serial.print("accel_mag = ");
  Serial.print(accel_mag);
  Serial.print(", gain = ");
  Serial.print(gain);
  Serial.print(", force = ");
  Serial.println(force);

  extenderServo.write(90);
}

