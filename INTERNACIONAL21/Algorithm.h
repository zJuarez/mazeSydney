#ifndef Algorithm_h
#define Algorithm_h

#include <Arduino.h>
#include <StackArray.h>
#include <QueueArray.h>
#include <SoftwareSerial.h>
#include "Priotities.h"
#include "SenseMap.h"



class Algorithm
{
  public: 

    Algorithm();
    Priotities robot{7, 7, 0, 'N'};
    SenseMap sensors;

    char camaraChar = '0';


    struct costs{
      byte a, b, node;
    };

    struct pointers{
      byte px, py;
    };
   
    void clear(byte ox, byte oy);
    void startBfs();
    void findWay();
    bool visualVictim();
    bool heatVictim();
    void rightTurn();
    void leftTurn();
    void moveForward(bool& perfect);
    byte objX, objY;
    void setup();
    
  private:

    int m4p[15][15];
    pointers save[15][15];
    bool algorithmVisiteds[15][15];
    char moves[100];
    byte BFSx, BFSy, contador, menor;
    int value;
    int reverseRightCount = 0;
};

#endif
