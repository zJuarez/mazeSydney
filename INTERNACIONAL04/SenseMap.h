#ifndef SenseMap_h
#define SenseMap_h

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Ultrasonic.h>
#include "Control.h"

class SenseMap
{
  public:
  
    SenseMap();
    Control motors;
    
    void setup();
    int getDistanceOf(uint8_t num);
    float firstTemperatureI;
    float firstTemperatureD;
    void checkDistances(bool);
    int teMamaste(double,char);
    void acomodo(double,double);
   
    
   private:
   
    int distance;    
    int uS;
};

#endif
