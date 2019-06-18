#ifndef SenseMap_h
#define SenseMap_h

#include <Arduino.h>
#include <i2cmaster.h>
#include <LiquidCrystal_I2C.h>
#include "Control.h"

class SenseMap
{
  public:
  
    SenseMap();
    Control motors;
    
    void setup();
    void beginTrig(const byte trig);
    float getDistanceOf(const byte echo, uint8_t num);
    float temperatureCelcius(int address);
    float firstTemperature;
    void checkDistances();
    
    const byte trig_E = 28;
    const byte echo_E = 29;
    const byte trig_A = 35;
    const byte echo_A = 34;
    const byte trig_D = 30;
    const byte echo_D = 31;
    const byte trig_I = 32;
    const byte echo_I = 33;
    int mlxLeft = 0x55<<1;    
    int mlxRight = 0x2A<<1; 
    
   private:
   
    unsigned long Time;
    const float sound = 34300.0;
    float distance;    
};

#endif
