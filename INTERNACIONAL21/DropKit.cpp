#include <Arduino.h>
#include "DropKit.h"

DropKit::DropKit(){}

void DropKit::setup()
{
  pinMode(RH_ENCODER_A, INPUT);
  pinMode(dispenserMotor1, OUTPUT);
  pinMode(dispenserMotor2, OUTPUT);
}

void DropKit::dropOneKitLeft()
{
  startTime = millis();

  rightCount = 0;

  while(rightCount < 1500)
    {
      digitalWrite(dispenserMotor1, LOW);
      analogWrite(dispenserMotor2, 200);

      if(millis() - startTime == 1000)
        rightCount = 1501;
    }

  this -> stopDispenser();
  rightCount = 0;

  while(rightCount < 300)
    {
      analogWrite(dispenserMotor1, 200);
      digitalWrite(dispenserMotor2, LOW);
    }

  this -> stopDispenser();
  rightCount = 0;
  startTime = millis();

  while(rightCount < 500)
    {
      digitalWrite(dispenserMotor1, LOW);
      analogWrite(dispenserMotor2, 200);

      if(millis() - startTime == 1000)
        rightCount = 1501;
    }

  this -> stopDispenser();
  rightCount = 0;

  while(rightCount < 1500)
    {
      analogWrite(dispenserMotor1, 200);
      digitalWrite(dispenserMotor2, LOW);
    }

  this -> stopDispenser();
}

void DropKit::dropOneKitRight()
{
  startTime = millis();
  rightCount = 0;

  while(rightCount < 500)
    {
      digitalWrite(dispenserMotor1, LOW);
      analogWrite(dispenserMotor2, 200);
    }

  stopDispenser();

  rightCount = 0;

  while(rightCount < 1500)
    {
      analogWrite(dispenserMotor1, 200);
      digitalWrite(dispenserMotor2, LOW);

      if(millis() - startTime == 1000)
        rightCount = 1501;
    }

  this -> stopDispenser();
  rightCount = 0;

  while(rightCount < 300)
    {
      digitalWrite(dispenserMotor1, LOW);
      analogWrite(dispenserMotor2, 200);
    }

  this -> stopDispenser();
  rightCount = 0;
  startTime = millis();

  while(rightCount < 500)
    {
      analogWrite(dispenserMotor1, 200);
      digitalWrite(dispenserMotor2, LOW);

      if(millis() - startTime == 1000)
        rightCount = 1501;
    }

  this -> stopDispenser();
  rightCount = 0;

  while(rightCount < 1500)
    {
      digitalWrite(dispenserMotor1, LOW);
      analogWrite(dispenserMotor2, 200);
    }

  this -> stopDispenser();
  rightCount = 0;

  while(rightCount < 500)
    {
      analogWrite(dispenserMotor1, 200);
      digitalWrite(dispenserMotor2, LOW);
    }

  stopDispenser();
}

void DropKit::dropTwoKitsLeft()
{
  startTime = millis();

  rightCount = 0;

  while(rightCount < 1500)
    {
      digitalWrite(dispenserMotor1, LOW);
      analogWrite(dispenserMotor2, 200);

      if(millis() - startTime == 1000)
        rightCount = 1501;
    }

  this -> stopDispenser();
  rightCount = 0;

  while(rightCount < 300)
    {
      analogWrite(dispenserMotor1, 200);
      digitalWrite(dispenserMotor2, LOW);
    }

  this -> stopDispenser();
  rightCount = 0;
  startTime = millis();

  while(rightCount < 500)
    {
      digitalWrite(dispenserMotor1, LOW);
      analogWrite(dispenserMotor2, 200);

      if(millis() - startTime == 1000)
        rightCount = 1501;
    }

  this -> stopDispenser();
  rightCount = 0;

  while(rightCount < 1500)
    {
      analogWrite(dispenserMotor1, 200);
      digitalWrite(dispenserMotor2, LOW);
    }

  this -> stopDispenser();
  startTime = millis();
  rightCount = 0;

  while(rightCount < 3000)
    {
      digitalWrite(dispenserMotor1, LOW);
      analogWrite(dispenserMotor2, 200);

      if(millis() - startTime == 1000)
        rightCount = 3001;
    }

  this -> stopDispenser();
  rightCount = 0;

  while(rightCount < 300)
    {
      analogWrite(dispenserMotor1, 200);
      digitalWrite(dispenserMotor2, LOW);
    }

  this -> stopDispenser();
  rightCount = 0;
  startTime = millis();

  while(rightCount < 500)
    {
      digitalWrite(dispenserMotor1, LOW);
      analogWrite(dispenserMotor2, 200);

      if(millis() - startTime == 1000)
        rightCount = 1501;
    }

  this -> stopDispenser();
  rightCount = 0;

  while(rightCount < 1500)
    {
      analogWrite(dispenserMotor1, 200);
      digitalWrite(dispenserMotor2, LOW);
    }

  this -> stopDispenser();
  rightCount = 0;

  while(rightCount < 500)
    {
      digitalWrite(dispenserMotor1, LOW);
      analogWrite(dispenserMotor2, 200);
    }

  stopDispenser();
}

void DropKit::dropTwoKitsRight()
{
  startTime = millis();

  rightCount = 0;

  while(rightCount < 1500)
    {
      analogWrite(dispenserMotor1, 200);
      digitalWrite(dispenserMotor2, LOW);

      if(millis() - startTime == 1000)
        rightCount = 1501;
    }

  this -> stopDispenser();
  rightCount = 0;

  while(rightCount < 300)
    {
      digitalWrite(dispenserMotor1, LOW);
      analogWrite(dispenserMotor2, 200);
    }

  this -> stopDispenser();
  rightCount = 0;
  startTime = millis();

  while(rightCount < 500)
    {
      analogWrite(dispenserMotor1, 200);
      digitalWrite(dispenserMotor2, LOW);

      if(millis() - startTime == 1000)
        rightCount = 1501;
    }

  this -> stopDispenser();
  rightCount = 0;

  while(rightCount < 1500)
    {
      digitalWrite(dispenserMotor1, LOW);
      analogWrite(dispenserMotor2, 200);
    }

  this -> stopDispenser();
  startTime = millis();
  rightCount = 0;

  while(rightCount < 3000)
    {
      analogWrite(dispenserMotor1, 200);
      digitalWrite(dispenserMotor2, LOW);

      if(millis() - startTime == 1000)
        rightCount = 3001;
    }

  this -> stopDispenser();
  rightCount = 0;

  while(rightCount < 300)
    {
      digitalWrite(dispenserMotor1, LOW);
      analogWrite(dispenserMotor2, 200);
    }

  this -> stopDispenser();
  rightCount = 0;
  startTime = millis();

  while(rightCount < 500)
    {
      analogWrite(dispenserMotor1, 200);
      digitalWrite(dispenserMotor2, LOW);

      if(millis() - startTime == 1000)
        rightCount = 1501;
    }

  this -> stopDispenser();
  rightCount = 0;

  while(rightCount < 1500)
    {
      digitalWrite(dispenserMotor1, LOW);
      analogWrite(dispenserMotor2, 200);
    }

  this -> stopDispenser();
  rightCount = 0;

  while(rightCount < 500)
    {
      analogWrite(dispenserMotor1, 200);
      digitalWrite(dispenserMotor2, LOW);
    }

  stopDispenser(); 
}

void DropKit::stopDispenser()
{
  digitalWrite(dispenserMotor1, HIGH);
  digitalWrite(dispenserMotor2, HIGH);
  delay(10);
  digitalWrite(dispenserMotor1, LOW);
  digitalWrite(dispenserMotor2, LOW);
}
