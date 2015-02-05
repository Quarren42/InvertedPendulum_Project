//. Motor driver shield- 2012 Copyright (c) Seeed Technology Inc.
// 
//  Original Author: Jimbo.we
//  Contribution: LG
//  
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

int pinIN1=8;//define IN1 interface
int pinIN2=11;//define IN2 interface 
int motorEnableA=9;//enable motor A
int motorSpeed = 127;
 
void setup()
{
  pinMode(pinIN1,OUTPUT);
  pinMode(pinIN2,OUTPUT);
  pinMode(motorEnableA,OUTPUT);
}
 
void forward()
{
     analogWrite(motorEnableA, motorSpeed);//input a simulation value to set the spee
     digitalWrite(pinIN2,LOW);//turn DC Motor A move anticlockwise
     digitalWrite(pinIN1,HIGH);
}
void backward()//
{
     analogWrite(motorEnableA, motorSpeed);//input a simulation value to set the speed
     digitalWrite(pinIN2,HIGH);//turn DC Motor A move clockwise
     digitalWrite(pinIN1,LOW);
}
void left()//
{
     analogWrite(motorEnableA, motorSpeed);//input a simulation value to set the speed
     digitalWrite(pinIN2,HIGH);//turn DC Motor A move clockwise
     digitalWrite(pinIN1,LOW);
}
void right()//
{
     analogWrite(motorEnableA, motorSpeed);//input a simulation value to set the speed
     digitalWrite(pinIN2,LOW);//turn DC Motor A move clockwise
     digitalWrite(pinIN1,HIGH);
}
void stop()//
{
     digitalWrite(motorEnableA,LOW);// Unenble the pin, to stop the motor. this should be done to avid damaging the motor. 
     delay(1000);
 
}

void loop()
{
  left();
  delay(2000);
  stop();
  right();
  delay(2000);
  stop();
 // delay(2000);
  forward();
  delay(2000);
  stop();
  backward();
  delay(2000); 
  stop(); 
}
