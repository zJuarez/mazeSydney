#ifndef DropKit_h
#define DropKit_h

#include <Arduino.h>

class DropKit
{
  public:
  
    DropKit();
    void setup();
    void dropOneKitRight();
    void dropOneKitLeft();
    void dropTwoKitsRight();
    void dropTwoKitsLeft();
    void stopDispenser();
    void rightEncoderEvent();
    
    const uint8_t RH_ENCODER_A = 19;
    volatile long rightCount = 0;
    
   private:
   
    const uint8_t dispenserMotor1  = 12;
    const uint8_t dispenserMotor2  = 13;
    unsigned long startTime;
};

#endif
