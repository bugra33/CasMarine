/*
 * Mate Rov Controller Program for  Mega 
 * Do not change Pressure Sensor Fluid Density
 *   Fresh Water: 997
 *   Sea Water: 1029
 *   
 *  0-7 joystick button
 *  8 joystick headswitch
 *  9 Joystick slider
 *  10 yaw
 *  11 x
 *  12 y
 *  13-24 ilk 4
 *  25-34 toggle
 *  35-36-37 slider
 */



 
#include <SPI.h>
#include <Ethernet.h>
#include <Servo.h>

Servo QuadMotor[8];

#include <Wire.h>

#include "MS5837.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define sideChangeDelay 10

byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 1, 177);
IPAddress serverIP(192, 168, 1, 34);
EthernetClient client;


double motorSpeed[8] = {0,0,0,0,0,0,0,0};


Adafruit_BNO055 bno = Adafruit_BNO055();
MS5837 sensor;


boolean motorsSide[] = {1,1,1,1,1,1,1,1};


byte relayPins1[] = {34,30,36,32,24,28,22,26};
byte relayPins2[] = {35,31,37,33,25,29,23,27};

//byte motorPins[] = {2,3,4,5,6,7,8,9};

byte motorPins[] = {7,9,6,4,8,2,3,5};

int inputData[38];
int outData[4];

int oldXval = 0;
int oldYval = 0;

//////////////////////////////HEIGHT PID VARIABLES
double heightErrSum;
double heightLastErr;
double heightLastTime;
double height_kp = 0.67;
double height_ki = 0.30;
double height_kd = 0.03;
double heightSetPoint = 0;

//////////////////////////////ROLL PID VARIABLES
double rollLastErr;
double rollErrSum;
double rollLastTime;
double roll_kp = 0.67;
double roll_ki = 0.30;
double roll_kd = 0.03;

//////////////////////////////PITCH PID VARIABLES
double pitchErrSum;
double pitchLastErr;
double pitchLastTime;
double pitch_kp = 0.67;
double pitch_ki = 0.30;
double pitch_kd = 0.03;

//////////////////////////////YAW PID VARIABLES
double yawErrSum;
double yawLastErr;
double yawLastTime;
double yaw_kp = 0.67;
double yaw_ki = 0.30;
double yaw_kd = 0.03;
double yawSetPoint = 0;
//////////////////////////////BALANCE VARIABLES
double rollSetPoint = 0;
double pitchSetPoint = 0;

Servo Gripper[3];


void setup(){
   Serial.begin(9600);
  for( int a = 0; a< sizeof(motorPins); a++){
    QuadMotor[a].attach( motorPins[a] ); 
    pinMode(relayPins1[a],OUTPUT);
    pinMode(relayPins2[a],OUTPUT);
  }
  setMotorSpeed(0,1000);
  setMotorSpeed(1,1000);
  setMotorSpeed(2,1000);
  setMotorSpeed(3,1000);
  setMotorSpeed(4,1000);
  setMotorSpeed(5,1000);
  setMotorSpeed(6,1000);
  setMotorSpeed(7,1000);
  delay(4000);
  setMotorSpeed(0,0);
  setMotorSpeed(1,0);
  setMotorSpeed(2,0);
  setMotorSpeed(3,0);
  setMotorSpeed(4,0);
  setMotorSpeed(5,0);
  setMotorSpeed(6,0);
  setMotorSpeed(7,0);
  delay(2000);
  
  Ethernet.begin(mac, ip);
  delay(3000);
  client.connect(serverIP, 5000);
  
  Wire.begin();
  IMU_initialize();
  for( int a = 0; a< sizeof(motorPins); a++){
    QuadMotor[a].attach( motorPins[a] ); 
    pinMode(relayPins1[a],OUTPUT);
    pinMode(relayPins2[a],OUTPUT);
  }
  Gripper[0].attach( 44 );
  Gripper[1].attach( 45 );
  Gripper[1].attach( 46 );
}

void loop() {

  
  balance();
  int xVal = 0;
  int yVal = 0;
  //yawBalance();
  //heightBalance();
  refreshMotorSpeeds();
  if(oldXval!= xVal){
    moveForward(-oldXval);
    moveForward(xVal);
    oldXval = xVal;
  }
  if(oldYval!= yVal){
    moveLR(-oldYval);
    moveLR(yVal);
    oldYval = yVal;
  }
  gripper(inputData[35],inputData[36]);
}


void IMU_initialize(){
  if(!bno.begin()){
    Serial.println("BNO055 cannot begin");
  }
  bno.setExtCrystalUse(true);
}

void refreshMotorSpeeds(){
  for( int a = 0; a < 8; a++ ){
   setMotorSpeed(a,motorSpeed[a]); 
  }
}
void stopAllMotors(){
  for( int a = 0; a < 8; a++ ){
   motorSpeed[a] = 0;
  }
}

void moveForward(int forwardSpeed){
  motorSpeed[4] += forwardSpeed;
  motorSpeed[5] -= forwardSpeed;
  motorSpeed[6] -= forwardSpeed;
  motorSpeed[7] += forwardSpeed;
}

void moveLR(int lrSpeed){
  motorSpeed[4] += lrSpeed;
  motorSpeed[5] += lrSpeed;
  motorSpeed[6] += lrSpeed;
  motorSpeed[7] += lrSpeed;
}



////////////////////////////////////////////////////Gripper

void gripper(int degree1, int degree2){
  Gripper[0].write(degree1);
  Gripper[1].write(degree2);
}

////////////////////////////////////////////////////Blue Robotics Pressure Sensor

void pressureSensorInitialize(){
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997);// kg/m^3 (freshwater, 1029 for seawater)
}

////////////////////////////////////////////////////MOVES

////////////////////////////////////////////////////BALANCE

void yawBalance(){
  double yawPIDval = yawPID(yawSetPoint,readYaw());
  motorSpeed[4] += yawPIDval;
  motorSpeed[5] -= yawPIDval;
  motorSpeed[6] -= yawPIDval;
  motorSpeed[7] += yawPIDval;
}


void balance(){
  //double initializePoint = setHeight();
  //////////////////////////////////////////////////ROLL BALANCE 
  double rollPIDval = rollPID(rollSetPoint,readRoll());
  //////////////////////////////////////////////////PITCH BALANCE
  double pitchPIDval = pitchPID(pitchSetPoint,readPitch());
  
  motorSpeed[0] +=  (-1) *  pitchPIDval/10000 - rollPIDval/10000;
  motorSpeed[1] +=  (-1) *  pitchPIDval/10000 + rollPIDval/10000;
  motorSpeed[2] +=  pitchPIDval/10000 + rollPIDval/10000;
  motorSpeed[3] +=  pitchPIDval/10000 - rollPIDval/10000;

}


void heightBalance(){
   sensor.read();
   double sensorValue = (double)sensor.depth();
   double heightPIDval = heightPID(heightSetPoint, sensorValue );
   motorSpeed[0] += heightPIDval;
   motorSpeed[1] += heightPIDval;
   motorSpeed[2] += heightPIDval;
   motorSpeed[3] += heightPIDval;
}

////////////////////////////////////////////MOTOR FUNCTIONS
void setMotorSpeed(int motorNumber, int motorSpeed ) {
  if( motorSpeed < 0 ){
    if( motorsSide[motorNumber] == 1 ){
      changeMotorSide(motorNumber,0);
    }
  }
  else{
    if( motorsSide[motorNumber] == 0 ){
      changeMotorSide(motorNumber,1);  
    }
  }
  motorSpeed = abs(motorSpeed);
  Serial.println(motorSpeed);
  QuadMotor[motorNumber].writeMicroseconds( limitSpeed( motorSpeed ) );
}
int limitSpeed( int givenSpeed ) {
  if( givenSpeed >= 1000 ) {
    return 2000;
  }
  else {
    return ( givenSpeed + 1000 );
  } 
}


void changeMotorSide(int motorNumber,boolean motorSide){
  //Serial.println("Motor side changed");
  setMotorSpeed(motorNumber, 0);
  if(motorSide == 1){
    digitalWrite(relayPins1[motorNumber], HIGH);
    digitalWrite(relayPins2[motorNumber], HIGH);
    motorsSide[motorNumber] = 1; 
  }   
  if(motorSide == 0){
    digitalWrite(relayPins1[motorNumber], LOW);
    digitalWrite(relayPins2[motorNumber], LOW);
    motorsSide[motorNumber] = 0; 
  }
}

//////////////////////////////////////////////////PID FUNCTIONS

double heightPID( double setPoint, double input){
  
  unsigned long startTime = millis();
  double timeGap = ( double ) ( startTime - heightLastTime );
  double error = setPoint - input;
  heightErrSum += ( error * timeGap );
  double dErr = ( error - heightLastErr ) / timeGap;
  double output = height_kp * error + height_ki * heightErrSum + height_kd * dErr;
  heightLastErr = error;
  heightLastTime = startTime;
  return output;  
}

double rollPID( double setPoint, double input){
  
  unsigned long startTime = millis();
  double timeGap = ( double ) ( startTime - rollLastTime );
  double error = setPoint - input;
  rollErrSum += ( error * timeGap );
  double dErr = ( error - rollLastErr ) / timeGap;
  double output = roll_kp * error + roll_ki * rollErrSum + roll_kd * dErr;
  rollLastErr = error;
  rollLastTime = startTime;
  return output;  
  
}

double pitchPID( double setPoint, double input){
  
  unsigned long startTime = millis();
  double timeGap = ( double ) ( startTime - pitchLastTime );
  double error = setPoint - input;
  pitchErrSum += ( error * timeGap );
  double dErr = ( error - pitchLastErr ) / timeGap;
  double output = pitch_kp * error + pitch_ki * pitchErrSum + pitch_kd * dErr;
  pitchLastErr = error;
  pitchLastTime = startTime;
  return output;  
  
}

double yawPID( double setPoint, double input){
  
  unsigned long startTime = millis();
  double timeGap = ( double ) ( startTime - yawLastTime );
  double error = setPoint - input;
  yawErrSum += ( error * timeGap );
  double dErr = ( error - yawLastErr ) / timeGap;
  double output = yaw_kp * error + yaw_ki * yawErrSum + yaw_kd * dErr;
  yawLastErr = error;
  yawLastTime = startTime;
  return output;  
  
}



/////////////////////////////////////IMU
double readRoll(){
  sensors_event_t event;
  bno.getEvent(&event);
  double roll = (double)event.orientation.z ;
  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  delay(BNO055_SAMPLERATE_DELAY_MS);
  return roll;
}

double readPitch(){
  sensors_event_t event;
  bno.getEvent(&event);
  double pitch = (double)event.orientation.y ;
  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  delay(BNO055_SAMPLERATE_DELAY_MS);
  return pitch;
}

double readYaw(){
  sensors_event_t event;
  bno.getEvent(&event);
  double yaw = (double)event.orientation.x ;
  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  delay(BNO055_SAMPLERATE_DELAY_MS);
  return yaw;
}

////////////////////////////////////////////////////COMMUNICATION

void readEthernet(){
  if(client.available()){
    for (int i = 0; i<sizeof(inputData); i++){
      inputData[i]=client.read();
      Serial.println(inputData[i]);
    }
  }
}
void sendEthernet(){
  for (int j = 0; j<sizeof(outData); j++){
    client.write(outData[j]);
    client.flush();
  }
  delay(10);
}
