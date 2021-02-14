/*
 * Version 0.2
 * Attempt to control the yaw with the gyro. Quaternions are a problem because they use the magnetometers.
 * When the motors turn on the magnetometers report bad values and the yaw angle becomes unusable
 * Pretty good results. a bit of overshoot and undershoot. tweaking kp and ki should result in straighter driving.
 *
 * Turning doesn't work properly for 180 degree turns. Had to fudge the turning to get 180s. This will need some work in the future.
 * Maybe a check of the magnetic azimuth with the motors turned off after a turn is done. then a tweak to get the proper rotation.
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
float yawActual = 0; //Yaw angle gets larger when car rotates counterclockwise from above.
float yawTarget;
float wz; // z axis gyro reading

float d=1;
int degRot, left, right, wv, rv;
float v, dt;
unsigned long millisOld, timerStart, calTimerStart;
float frontDuration, frontObsDistance;
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
  v = 1.1;
  wv = (v + 0.07213) / 0.00836129;
  left = wv;
  right = wv;
  setSpeed(left, right);
  imu::Vector<3> gyr =myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  dt = (millis() - millisOld) / 1000.;
  millisOld = millis();
  wz = gyr.z();
  yawActual = yawActual + wz * dt;
  

  delay(500);
  forward(5, v);
  delay(300);
//  turnRight(170, wv);
//  delay(300);
//  forward(5,v);
//  delay(300);
//  turnRight(170, wv);
  
} // ENDING BRACE for loop()



// Begin Function Declarations
void setSpeed(int leftVal,int rightVal){
  if(leftVal > 255){
    leftVal = 255;
  }
    if(leftVal < 100){
    leftVal = 100;
  }
  if(rightVal > 255){
    rightVal = 255;
  }
    if(rightVal < 100){
    rightVal = 100;
  }
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
  float kp = 25, ki = 0, kd=0;
  float kCorrection;
  float yawErrorSum = 0;

    
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
    wz = gyr.z();
    yawActual = yawActual + wz * dt;
    yawError = yawTarget - yawActual;
    yawErrorSum += yawError;
    

    //drifting right, speed up right wheels.
    //drifting left, slow down right wheel.
    //wz is negative when rotating clockwise. 
    //if wz is negative we need a positive derivative term...minus sign for kd*wz.
    //ENA controls the left side.
    right = wv + kp*yawError + ki*yawErrorSum - kd * wz;
    setSpeed(left, right);
      
    
    
    Serial.print(wz);
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
  //Add forward control here but may need to change signs to get things to work in reverse.
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
  float angleTurned = 0;
  float t;
  //Set speed to 110 for turning. Then use wv to set back to whatever speed we were at prior to turning.
  setSpeed(100, 100);

  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  millisOld = millis();
  while(angleTurned < deg){
    imu::Vector<3> gyr =myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    dt = (millis() - millisOld) / 1000.;
    millisOld = millis();
    angleTurned += -gyr.z()*dt; //adding negative values because turning right will give us negative omega for gyr.z()
    
    
}

// t= (deg + 0.9) / 100.119 * 1000; calculation to get the desired angle of turn.
// Hoping to take this out and just use the gyro reading to determing how far we have turned.
//delay(t);
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
