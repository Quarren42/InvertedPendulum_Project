#include <PID_AutoTune_v0.h>
#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Encoder.h>

#include "utility/Adafruit_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Encoder myEnc(2, 3);

Adafruit_DCMotor *myMotor = AFMS.getMotor(1);

const int leftLimitSwitch = 4;
const int rightLimitSwitch = 5;
  
double Kp, Ki, Kd, Pu, Ku, KpIn, KiIn, KdIn;
int tempSpeed;
double Setpoint, Input, Output;
long time, oldTime;

//Tu = 0.27 //Tu is the oscillation period
//Ku = 50 //Ku is the ultimate gain (value of P used to get the constant oscillation)
//Kp = 0.6 * Ku; //30
//Ki = (2 * Kp / Tu);  //222.22
//Kd = (Kp * Tu) / 8; //1.0125

//^^^ all for PID controller

//Tu = 0.27 //Tu is the oscillation period
//Ku = 50 //Ku is the ultimate gain (value of P used to get the constant oscillation)
//Kp = 0.8 * Ku; //40
//Kd = (Kp * Tu) / 8; //1.35

//^^^ all for PD controller


#define Kp  65 //50 for tuning
#define Ki  0
#define Kd  0
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
 
// PID_ATune aTune(&Input, &Output);

int value = 877;
char buffer[5];
void setup() 
{
  AFMS.begin();
  
  pinMode(leftLimitSwitch, INPUT_PULLUP);
  pinMode(rightLimitSwitch, INPUT_PULLUP);

  Serial.begin(9600);
  
 myPID.SetOutputLimits(-255, 255);
 myPID.SetMode(AUTOMATIC);
 myPID.SetSampleTime(50);
  
Setpoint = 0;

myMotor->setSpeed(255);

}

long oldPosition  = -999;

void loop()
{
  
  
 long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Input = (newPosition - 2048);
  }
  
 // Serial.print("#S|LOGTEST|[");
//Serial.print(itoa((Input), buffer, 10));
//Serial.println("]#");

 time = millis();
    
Serial.print(Input);
//Serial.print(", ");
//Serial.print(time);
Serial.print('\n'); 
  
   
  //myPID.SetTunings(Kp, Ki, Kd);
  myPID.Compute(); 
  
  while ((digitalRead(leftLimitSwitch)) == LOW || (digitalRead(rightLimitSwitch) == LOW)){
   myMotor->setSpeed(0);
    myMotor->run(RELEASE);
  }
  
 /* else if (digitalRead(rightLimitSwitch) == LOW){
   myMotor->setSpeed(0);
    myMotor->run(RELEASE);
  } */
   
  if (Output > 0){
    tempSpeed = abs(Output); 
     myMotor->setSpeed(tempSpeed);
  myMotor->run(BACKWARD);
  }
  
  if (Output < 0){
    tempSpeed = abs(Output);
     myMotor->setSpeed(tempSpeed);
    myMotor->run(FORWARD);
  }
}
