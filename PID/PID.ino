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

//Tu = 0.357167 //Tu is the oscillation period
//Ku = 0.333 //Ku is the ultimate gain (value of P used to get the constant oscillation)
//Kp = 0.6 * Ku; //0.1998
//Ki = (2 * Kp / Tu);  //1.118804
//Kd = (Kp * Tu) / 8; //0.071361

//^^^ all for PID controller

//Tu = 0.27 //Tu is the oscillation period
//Ku = 50 //Ku is the ultimate gain (value of P used to get the constant oscillation)
//Kp = 0.8 * Ku; //40
//Kd = (Kp * Tu) / 8; //1.35

//^^^ all for PD controller


#define Kp 0.333 //0.1988  0.33 - 0.34 (0.3 - 0.4) lower than 0.331, above 0.33
#define Ki  0 //1.1188043688 18.556701
#define Kd  0 //0.0713619666 0.0213885

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int value = 877;
char buffer[5];
void setup() 
{
  
  myservo.attach(9);

  pinMode(leftLimitSwitch, INPUT_PULLUP);
  pinMode(rightLimitSwitch, INPUT_PULLUP);

  Serial.begin(9600);

  myPID.SetOutputLimits(-25, 25); //range from -25 to 25
  myPID.SetMode(AUTOMATIC);      //motor starts at 12 speed, so add/subtract 12
  myPID.SetSampleTime(10);

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
 // Serial.print(", ");
  //Serial.print(time);
  //Serial.print(Output);
  //Serial.print(", ");
  //Serial.print(tempSpeed);
  Serial.print('\n'); 

  myPID.Compute(); 

  while ((digitalRead(leftLimitSwitch)) == LOW || (digitalRead(rightLimitSwitch) == LOW)){
    stop();
  }
  
   tempSpeed = Output + 90;  //add the limit of the PID equation; 90 for full speed, 45 for half speed
   
   if (tempSpeed > 90){
     tempSpeed = tempSpeed + 12;
   }
   
   else if (tempSpeed < 90){
     tempSpeed = tempSpeed - 12;
   }  
     
  
  
  myservo.write(tempSpeed);
  
}

