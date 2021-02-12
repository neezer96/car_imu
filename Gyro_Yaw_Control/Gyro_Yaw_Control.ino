/*
 * Version 0.1
 * Attempt to control the yaw with the gyro. Quaternions are a problem because they use the magnetometers.
 * When the motors turn on the magnetometers report bad values and the yaw angle becomes unusable
 * 
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
#define ledPin 13 //used to check calibration of IMU. LEd will light when system and accel calibration is good.

// Initialize IMU
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 myIMU = Adafruit_BNO055();

float rollActual;
float pitchActual;
float yawActual = 0.; //Yaw angle gets larger when car rotates counterclockwise from above.
float yawTarget;

float d=1;
int degRot, left, right, wv, rv;
float v;
unsigned long millisOld, timerStart, calTimerStart;
float dt;
float frontDuration, frontObsDistance;
char cmd;
unsigned long calTime = 90000; // miliseconds for calibration. 2 minutes should be enough to get a good calibration.
bool calibrated = 0;





void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
Serial.println("before begin");
myIMU.begin();
Serial.println("IMU has started");
delay(1000);
int8_t temp = myIMU.getTemp();



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

millisOld = millis();
}

void loop() {
  uint8_t system, gyro, accel, mg = 0;
  myIMU.getCalibration(&system, &gyro, &accel, &mg);
  if(accel!=3 | gyro!=3 | mg!=3 | system!=3){
    calibrated=0;
    digitalWrite(ledPin, LOW);
    delay(1500);
  }
  //calibrate();
  v = 0.8;
  wv = (v + 0.07213) / 0.00836129;
  left = wv;
  right = wv;
  setSpeed(left, right);
  imu::Vector<3> gyr =myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  dt = (millis() - millisOld) / 1000.;
  millisOld = millis();
  yawActual = yawActual + gyr.z() * dt;

  delay(5000);
  forward(5, v);
} // ENDING BRACE for loop()



// Begin Function Declarations
void setSpeed(int leftVal,int rightVal){
  analogWrite(ENA,leftVal);
  analogWrite(ENB,rightVal);
}
void calibrate(){
  Serial.print("calibrating");
  uint8_t system, gyro, accel, mg = 0;
  while(!calibrated){
    myIMU.getCalibration(&system, &gyro, &accel, &mg);
    Serial.print(accel);
    Serial.print(",");
    Serial.print(gyro);
    Serial.print(",");
    Serial.print(mg);
    Serial.print(",");
    Serial.println(system);

    if(accel==3 && gyro==3 && mg==3 && system==3){
      calibrated=1;
      digitalWrite(ledPin, HIGH);
      delay(1500);
    }

    
    delay(BNO055_SAMPLERATE_DELAY_MS);
  }
}

void forward(float d, float v){
  yawTarget = yawActual;
  float t, yawError;
  float kp = 0.2;
  float kCorrection;
    
  t=d/v*1000; // time to run motors in miliseconds to achieve desired distance.
  timerStart = millis();
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  millisOld = millis();
  while((millis() - timerStart) < t){
    imu::Vector<3> gyr =myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    dt = (millis() - millisOld) / 1000.;
    millisOld = millis();
    yawActual = yawActual + gyr.z() * dt;
    yawError = yawTarget - yawActual;
    kCorrection = yawError * kp; 

    if(yawError > 0){
      //slow down left wheels.
      //ENA controls the left side.
      left = wv - kp*yawError;
      setSpeed(left, right);
      
    }
    if(yawError < 0){
      //Speed up left wheels.
      left = wv + kp*yawError;
      setSpeed(left, right);
    }


    Serial.print(kCorrection);
    Serial.print(", ");
    Serial.print(yawActual);
    Serial.print(", ");
    Serial.print(yawTarget);
    Serial.print(", ");
    Serial.print(yawError);    
    Serial.print(", ");
    Serial.print(left);
    Serial.print(", ");
    Serial.println(right);
    
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
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
