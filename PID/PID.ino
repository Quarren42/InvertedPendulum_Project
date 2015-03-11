#include <PID_v1.h>
#include <Wire.h>
#include <Encoder.h>
#include <Servo.h>

Servo myservo;

Encoder myEnc(2, 3);

const int leftLimitSwitch = 4;
const int rightLimitSwitch = 5;

double Kp, Ki, Kd, Pu, Ku, KpIn, KiIn, KdIn;
int tempSpeed;
double Setpoint, Input, Output;
long time, oldTime;

//Tu = 0.1358 //Tu is the oscillation period
//Ku = 2.1 //Ku is the ultimate gain (value of P used to get the constant oscillation)
//Kp = 0.6 * Ku; //1.26
//Ki = (2 * Kp / Tu);  //18.5567010309278
//Kd = (Kp * Tu) / 8; //0.0213885

//^^^ all for PID controller

//Tu = 0.27 //Tu is the oscillation period
//Ku = 50 //Ku is the ultimate gain (value of P used to get the constant oscillation)
//Kp = 0.8 * Ku; //40
//Kd = (Kp * Tu) / 8; //1.35

//^^^ all for PD controller


#define Kp 1.26  //2.1, 1.65
#define Ki  18.556701
#define Kd  0.0213885
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// PID_ATune aTune(&Input, &Output);

int value = 877;
char buffer[5];
void setup() 
{
  
  myservo.attach(9);

  pinMode(leftLimitSwitch, INPUT_PULLUP);
  pinMode(rightLimitSwitch, INPUT_PULLUP);

  Serial.begin(9600);

  myPID.SetOutputLimits(-45, 45); //range from 0-180; use -90 to 90 because the encoder measures -2048 to 2048
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(50);

  Setpoint = 0;

}

void stop()//
{
  myservo.write(90);
}

long oldPosition  = -999;

void loop()
{


  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Input = (newPosition - 2048);
  }

  time = millis();

  Serial.print(Input);
 Serial.print(", ");
  //Serial.print(time);
 Serial.print(Output);
 // Serial.print(", ");
  //Serial.print(tempSpeed);
  Serial.print('\n'); 

  myPID.Compute(); 

  while ((digitalRead(leftLimitSwitch)) == LOW || (digitalRead(rightLimitSwitch) == LOW)){
    stop();
  }
  
   tempSpeed = Output + 45; //add the limit of the PID equation; 90 for full speed, 45 for half speed
  myservo.write(tempSpeed);
  
}

