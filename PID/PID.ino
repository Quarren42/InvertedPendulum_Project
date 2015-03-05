#include <PID_AutoTune_v0.h>
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


#define Kp 0.7  //50 for tuning
#define Ki  0
#define Kd  0
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

  myPID.SetOutputLimits(-65, 65); //0 and 180 draw too much power
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

  // Serial.print("#S|LOGTEST|[");
  //Serial.print(itoa((Input), buffer, 10));
  //Serial.println("]#");

  time = millis();

  Serial.print(Input);
 // Serial.print(", ");
  //Serial.print(time);
 // Serial.print(Output);
 // Serial.print(", ");
  //Serial.print(tempSpeed);
  Serial.print('\n'); 


  //myPID.SetTunings(Kp, Ki, Kd);
  myPID.Compute(); 

  while ((digitalRead(leftLimitSwitch)) == LOW || (digitalRead(rightLimitSwitch) == LOW)){
    stop();
  }
  
   tempSpeed = Output + 90;
  myservo.write(tempSpeed);
  
}

