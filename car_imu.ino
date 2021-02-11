/*
 * Version 0.1
 * Version 0.2 Added yaw measurement to forward function. Removed bluetooth options until ready.
 * yaw from quaternions is influenced by magnetometers. When motors are turned on yaw angle gets all wierd.
 * Need to use gyros only instead.
 * 
 */

/*
 * Libraries needed for 
 */

// Setup for IMU
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

//ultrasonic declarations
#define frontTriggerPin 2
#define frontEchoPin 4
#define maxDistance 400
#define iterations 5

//Motor Control declaration
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

// Initialize IMU
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 myIMU = Adafruit_BNO055();

//quaternion initialization
float q0;
float q1;
float q2;
float q3;

float rollActual;
float pitchActual;
float yawActual;
float yawTarget;



int ledPin = 13; //used to check calibration of IMU. LEd will light when system and accel calibration is good.

float d=1;
int degRot, left, right, wv, rv;
float v;
unsigned long millisOld, timerStart, calTimerStart;
float frontDuration, frontObsDistance;
char cmd;
unsigned long calTime = 90000; // miliseconds for calibration. 2 minutes should be enough to get a good calibration.
bool calibrated = 0;


void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
Serial.println("before begin");
myIMU.begin();
Serial.println("IMU has started");
delay(1000);
int8_t temp = myIMU.getTemp();
millisOld = millis();

pinMode(ENA,OUTPUT);
pinMode(ENB,OUTPUT);
pinMode(IN1,OUTPUT);
pinMode(IN2,OUTPUT);
pinMode(IN3,OUTPUT);
pinMode(IN4,OUTPUT);
digitalWrite(ENA,HIGH);
digitalWrite(ENB,HIGH);
pinMode(ledPin, OUTPUT);
digitalWrite(ledPin, LOW);
}

void loop() {
  v = 0.8;
  wv = (v + 0.07213) / 0.00836129;
  left = wv;
  right = wv;
  setSpeed(left, right);
  uint8_t system, gyro, accel, mg = 0;

  // Calibration Loop Run for 1 minute before moving on.
  delay(2000);
  calTimerStart = millis();
  while(!calibrated){
    myIMU.getCalibration(&system, &gyro, &accel, &mg);
    imu::Quaternion quat=myIMU.getQuat();
 
    q0=quat.w();
    q1=quat.x();
    q2=quat.y();
    q3=quat.z();

    rollActual=atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
    pitchActual=asin(2*(q0*q2-q3*q1));
    yawActual = atan2(2.0*(q3* q0 + q1 * q2) , - 1.0 + 2.0 * (q0 * q0 + q1 * q1));

 
    rollActual=rollActual/(2*3.141592654)*360;
    pitchActual=pitchActual/(2*3.141592654)*360;
    yawActual = yawActual / (2*3.141592654) * 360;

    // convert yaw angles from -180 to 180 to 0-360
    if(yawActual < 0){
      yawActual = 360 + yawActual;
    }

    Serial.print(millis() - calTimerStart);
    Serial.print(", ");
    Serial.print(yawActual);
    Serial.print(", ");
    Serial.print(accel);
    Serial.print(", ");
    Serial.print(gyro);
    Serial.print(", ");
    Serial.print(mg);
    Serial.print(", ");
    Serial.println(system);


    if(gyro == 3 && accel == 3 && mg == 3){
      digitalWrite(ledPin, HIGH);
      calibrated = 1;
      Serial.println("Calibration Complete");
      
    }
    if(gyro != 3 || accel != 3 || mg != 3){
      digitalWrite(ledPin, LOW);
      calibrated = 0;
    }

    delay(BNO055_SAMPLERATE_DELAY_MS);
  }



delay(5000);
forward(10, v, yawActual);
delay(5000);


} // ENDING BRACE for loop()



// Begin Function Declarations
void setSpeed(int leftVal,int rightVal){
  analogWrite(ENA,leftVal);
  analogWrite(ENB,rightVal);
}


void forward(float d, float v, float yawTarget){
  float t, yawError;
  t=d/v*1000; // time to run motors in miliseconds to achieve desired distance.
  millisOld = millis();
  
  // while in forward loop we need to get the IMU data each loop.
  imu::Quaternion quat=myIMU.getQuat();  
  q0=quat.w();
  q1=quat.x();
  q2=quat.y();
  q3=quat.z();
  rollActual=atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
  pitchActual=asin(2*(q0*q2-q3*q1));
  yawActual = atan2(2.0*(q3* q0 + q1 * q2) , - 1.0 + 2.0 * (q0 * q0 + q1 * q1));
  
  if(yawActual < 0){
    yawActual = 360 + yawActual;
  }
  

  timerStart = millis();
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  while((millis() - timerStart) < t){
    // Get IMU data while going forward.
    imu::Quaternion quat=myIMU.getQuat();  
    q0=quat.w();
    q1=quat.x();
    q2=quat.y();
    q3=quat.z();
    rollActual=atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
    pitchActual=asin(2*(q0*q2-q3*q1));
    yawActual = atan2(2.0*(q3* q0 + q1 * q2) , - 1.0 + 2.0 * (q0 * q0 + q1 * q1));
    if(yawActual < 0){
      yawActual = 360 + yawActual;
    }
    yawError = yawTarget - yawActual;
    Serial.print(yawActual);
    Serial.print(", ");
    Serial.print(yawTarget);
    Serial.print(", ");
    Serial.println(yawError);    

  
}

  stopCar();
}


void backward(float d, float v){
float t;
digitalWrite(IN1,LOW);
digitalWrite(IN2,HIGH);
digitalWrite(IN3,HIGH);
digitalWrite(IN4,LOW);
t=d/v*1000;
delay(t);
stopCar();
}


void turnRight(float deg, int wv){
  float t;
//Set speed to 125 for turning.
setSpeed(125, 125);

digitalWrite(IN1,HIGH);
digitalWrite(IN2,LOW);
digitalWrite(IN3,HIGH);
digitalWrite(IN4,LOW);
t= (deg + 0.9) / 100.119 * 1000;
delay(t);
stopCar();

//reset speed
setSpeed(wv, wv);
}


void turnLeft(float deg, int wv){
  float t;
//set speed to 125 for turning.
setSpeed(125, 125);

digitalWrite(IN1,LOW);
digitalWrite(IN2,HIGH);
digitalWrite(IN3,LOW);
digitalWrite(IN4,HIGH);
t= (deg + 0.9) / 100.119 * 1000;
delay(t);
stopCar();
setSpeed(wv, wv);
}


void stopCar(){
digitalWrite(IN1,LOW);
digitalWrite(IN2,LOW);
digitalWrite(IN3,LOW);
digitalWrite(IN4,LOW);
}

void calF(){
digitalWrite(IN1,HIGH);
digitalWrite(IN2,LOW);
digitalWrite(IN3,LOW);
digitalWrite(IN4,HIGH);
delay(5000);
stopCar();
}
void calB(){
digitalWrite(IN1,LOW);
digitalWrite(IN2,HIGH);
digitalWrite(IN3,HIGH);
digitalWrite(IN4,LOW);
delay(5000);
stopCar();
}
void calR(int wv){
stopCar();
//set turning speed. Use low power to avoid skidding on power up and down.
setSpeed(150, 150);

digitalWrite(IN1,HIGH);
digitalWrite(IN2,LOW);
digitalWrite(IN3,HIGH);
digitalWrite(IN4,LOW);
delay(2000);
stopCar();

//reset motor speed to wv after turn.
setSpeed(wv, wv);
}
void calL(){
digitalWrite(IN1,LOW);
digitalWrite(IN2,HIGH);
digitalWrite(IN3,LOW);
digitalWrite(IN4,HIGH);
delay(5000);
stopCar();
}
