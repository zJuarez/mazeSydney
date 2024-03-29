#ifndef Movement_h
#define Movement_h


#include <Arduino.h>
#include "Algorithm.h"

class Movement
{
   public:

    Movement();
    Algorithm maze;
    
    void moveOn();
    void isBlack();
    void responseNoBlack();
    void responseBlack();
    void conditionOne();
    void conditionTwo();
    void conditionThree();
    void conditionFour(bool);
    void setup();
    bool yaPasamos   = false;

    byte xA, yA, zA, xF, yF;
    bool endOfRound = false;
    char green = '0';
    bool alSuicidio = false;
    bool salidaDeChinos = false;
    int xChina, yChina;
    bool tumbada = false;

   private:

   int countPendings = 0;
   bool black;
   bool decision = false;
   int dF, dD, dI;
};

#endif 
