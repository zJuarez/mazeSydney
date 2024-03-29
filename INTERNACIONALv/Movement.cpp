#include <Arduino.h>
#include "Movement.h"

Movement::Movement(){}

  bool perfect = true;

void Movement::setup()
{
  Serial3.begin(115200);
}

void Movement::moveOn()
{
  delay(10);
  int d = maze.sensors.getDistanceOf(3);

  if(Serial3.available() > 0)
    green = Serial3.read();

  if((green == 'a' || green == 'k' || millis() > 240000) && (alSuicidio))
  {
    salidaDeChinos = true;
    this -> conditionFour(salidaDeChinos);
  }
  
  
  if(d<21)
  {
    maze.robot.correction(maze.robot.getDirection());
    this-> conditionOne();
  }
  else{
   if(maze.sensors.motors.banderaTotalVision || maze.sensors.motors.banderaTotalCalor)
   maze.robot.tile[maze.robot.x][maze.robot.y][maze.robot.z].victim(1);
   maze.moveForward(perfect);
   if(perfect)
   this -> isBlack();
  }
   
}

void Movement::isBlack()
{
  //maze.sensors.motors.escribirLCD("Revisando Color", "de cuadro");
  ////delay(500);
    
  if(maze.sensors.motors.cuadroVerde() && !yaPasamos)
  {
    switch(maze.robot.getDirection())
        {
          case 'N': maze.robot.x--;
          break;
          case 'E': maze.robot.y++;
          break;
          case 'W': maze.robot.y--;
          break;
          case 'S': maze.robot.x++;
          break;
        }

  delay(10);
  dD = maze.sensors.getDistanceOf(2);
  delay(10);
  dI = maze.sensors.getDistanceOf(1);
  
  maze.robot.changeSquare(&maze.robot.tile[maze.robot.x][maze.robot.y][maze.robot.z], dD, dI, 60, maze.robot.getDirection());
  maze.robot.changeStatus(&maze.robot.tile[maze.robot.x][maze.robot.y][maze.robot.z], maze.robot.getDirection());
        
        maze.sensors.motors.rightCount = 0;
        while(maze.sensors.motors.rightCount < 1400)
        maze.sensors.motors.moveAdelanteFast();

        maze.sensors.motors.detenerse();
        delay(200);

        yaPasamos = true;

        switch(maze.robot.getDirection())
        {
          case 'N': maze.robot.x--;
          break;
          case 'E': maze.robot.y++;
          break;
          case 'W': maze.robot.y--;
          break;
          case 'S': maze.robot.x++;
          break;
        }

        delay(10);
  dF = maze.sensors.getDistanceOf(3);
  delay(10);
  dD = maze.sensors.getDistanceOf(2);
  delay(10);
  dI = maze.sensors.getDistanceOf(1);
  
  maze.robot.changeSquare(&maze.robot.tile[maze.robot.x][maze.robot.y][maze.robot.z], dD, dI, dF, maze.robot.getDirection());
  maze.robot.changeStatus(&maze.robot.tile[maze.robot.x][maze.robot.y][maze.robot.z], maze.robot.getDirection());

  Serial3.write('f');

  this -> conditionOne();
    }
 else
  (maze.sensors.motors.cuadroNegro() && !maze.sensors.motors.bumperCabron()) ? this -> responseBlack() : this -> responseNoBlack();  
}

void Movement::responseNoBlack()
{
  //maze.sensors.motors.escribirLCD("No es", "cuadro negro");
  //delay(500);
 // int hola = 0;
 
  maze.robot.changeCoordinates(maze.robot.getDirection());
  
  /*if(hola == 1)
    maze.sensors.motors.escribirLCD("TRANSFIRIENDO", "ARRIBA");
  else if(hola == 2)
    maze.sensors.motors.escribirLCD("TRANSFIRIENDO", "ABAJO");
  else if(hola == 3)
    maze.sensors.motors.escribirLCD("TRANSFIRIENDO", "DERECHA");
  else if(hola == 4)
    maze.sensors.motors.escribirLCD("TRANSFIRIENDO", "IZUIERDA");
  else
    maze.sensors.motors.escribirLCD("NADA", "NADA");
  delay(1500);*/
  //maze.sensors.motors.escribirLetraLCD(maze.robot.getDirection());
  //delay(500);

  delay(10);
  dF = maze.sensors.getDistanceOf(3);
  delay(10);
  dD = maze.sensors.getDistanceOf(2);
  delay(10);
  dI = maze.sensors.getDistanceOf(1);
  
  maze.robot.changeSquare(&maze.robot.tile[maze.robot.x][maze.robot.y][maze.robot.z], dD, dI, dF, maze.robot.getDirection());
  
  //maze.sensors.motors.escribirNumLCD(maze.robot.x);
  //delay(500);
 //maze.sensors.motors.escribirNumLCD(maze.robot.y);
 //delay(500);
 // maze.sensors.motors.escribirLCD("EMPEZO EN", " ");
 // delay(500);
 // maze.sensors.motors.escribirNumLCD(maze.robot.startX);
  //delay(500);
  //maze.sensors.motors.escribirNumLCD(maze.robot.startY);
  //delay(500);
 //maze.sensors.motors.escribirNumLCD(maze.robot.z);
 //delay(500);

  //maze.sensors.motors.printLoc(maze.robot.x,maze.robot.y,maze.robot.z);
 //delay(800);

// maze.sensors.motors.printLoc(maze.robot.x,maze.robot.y,maze.robot.z);
 //delay(1000);
 
  maze.robot.changeStatus(&maze.robot.tile[maze.robot.x][maze.robot.y][maze.robot.z], maze.robot.getDirection());

  this -> conditionOne();
}

void Movement::responseBlack()
{
  //maze.sensors.motors.escribirLCD("Caudro negro", "detectado");

  maze.sensors.motors.rightCount = 0;
  maze.sensors.motors.girosX=3;
  

  while(maze.sensors.motors.rightCount < maze.sensors.motors.tic1)
    maze.sensors.motors.atrasPID(maze.sensors.motors.de);

  maze.sensors.motors.detenerse();
  delay(200);

  switch(maze.robot.getDirection())
  {
    case 'N':
    {
      maze.robot.tile[maze.robot.x-1][maze.robot.y][maze.robot.z].blackTile(1);
      maze.robot.tile[maze.robot.x-1][maze.robot.y][maze.robot.z].visited(1);
      maze.robot.tile[maze.robot.x-1][maze.robot.y][maze.robot.z].pending(0);
    } break;
    case 'E':
    {
      maze.robot.tile[maze.robot.x][maze.robot.y+1][maze.robot.z].blackTile(1);
      maze.robot.tile[maze.robot.x][maze.robot.y+1][maze.robot.z].visited(1);
      maze.robot.tile[maze.robot.x][maze.robot.y+1][maze.robot.z].pending(0);
    }  break;
    case 'W':
    {
      maze.robot.tile[maze.robot.x][maze.robot.y-1][maze.robot.z].blackTile(1);
      maze.robot.tile[maze.robot.x][maze.robot.y-1][maze.robot.z].visited(1);
      maze.robot.tile[maze.robot.x][maze.robot.y-1][maze.robot.z].pending(0);
    } break;
    case 'S':
    {
      maze.robot.tile[maze.robot.x+1][maze.robot.y][maze.robot.z].blackTile(1);
      maze.robot.tile[maze.robot.x+1][maze.robot.y][maze.robot.z].visited(1);
      maze.robot.tile[maze.robot.x+1][maze.robot.y][maze.robot.z].pending(0);
    } break;
  }

  this -> conditionOne();
}

void Movement::conditionOne()
{
  //maze.sensors.motors.escribirLCD("Entrando", "a condicion 1");
  //delay(500);

  (maze.robot.correctMovementRight(maze.robot.getDirection())) ? maze.rightTurn(0) : this -> conditionTwo();
}
void Movement::conditionTwo()
{
  //maze.sensors.motors.escribirLCD("Entrando", "a condicion 2");
  //delay(500);

  if(maze.robot.correctMovementFront(maze.robot.getDirection()))
  {}
  else
    this -> conditionThree();
}

void Movement::conditionThree()
{

  //maze.sensors.motors.escribirLCD("Entrando", "a condicion 3");
  //delay(500);

  (maze.robot.correctMovementLeft(maze.robot.getDirection())) ? maze.leftTurn(0) : this -> conditionFour(salidaDeChinos);
}

void Movement::conditionFour(bool chinitos)
{
  //maze.sensors.motors.escribirLCD("Entrando", "a condicion 4");
  //delay(500);
  if(!chinitos)
  {
  countPendings = 0;

  if(maze.robot.z == 0)
  {
    for(int i = maze.robot.firstCount - 1; i >= 0; i--)
    {
      if(!maze.robot.tile[maze.robot.fFxs[i]][maze.robot.fFys[i]][0].visited() && maze.robot.tile[maze.robot.fFxs[i]][maze.robot.fFys[i]][0].exist())
      {
          xA = maze.robot.fFxs[i];
          yA = maze.robot.fFys[i];
          countPendings++;
          i = 0;
      }
    }
  }
  else if(maze.robot.z == 1)
  {
    for(int i = maze.robot.secondCount - 1; i >= 0; i--)
    {
      if(!maze.robot.tile[maze.robot.sFxs[i]][maze.robot.sFys[i]][1].visited() && maze.robot.tile[maze.robot.sFxs[i]][maze.robot.sFys[i]][1].exist())
      {
        xA = maze.robot.sFxs[i];
        yA = maze.robot.sFys[i];
        countPendings++;
        i = 0;
      }
    }
  }
  else if(maze.robot.z == 2)
  {
    for(int i = maze.robot.thirdCount - 1; i >= 0; i--)
    {
      if(!maze.robot.tile[maze.robot.tFxs[i]][maze.robot.tFys[i]][2].visited() && maze.robot.tile[maze.robot.tFxs[i]][maze.robot.tFys[i]][2].exist())
      {
        xA = maze.robot.tFxs[i];
        yA = maze.robot.tFys[i];
        countPendings++;
        i = 0;
      }
    }
  }
  else if(maze.robot.z == 3)
  {
    for(int i = maze.robot.fourthCount - 1; i >= 0; i--)
    {
      if(!maze.robot.tile[maze.robot.fhFxs[i]][maze.robot.fhFys[i]][3].visited() && maze.robot.tile[maze.robot.fhFxs[i]][maze.robot.fhFys[i]][3].exist())
      {
        xA = maze.robot.fhFxs[i];
        yA = maze.robot.fhFys[i];
        countPendings++;
        i = 0;
      }
    }
  }

  if(countPendings == 0)
  {
    xA = maze.robot.startX;
    yA = maze.robot.startY;
    zA = maze.robot.startZ;
    endOfRound = true;
    decision = false;

    for(int i = 0; i < 15; i++)
    {
      for(int j = 0; j < 15; j++)
      {
        if(maze.robot.tile[i][j][maze.robot.z].upRamp() && !maze.robot.tile[i][j][maze.robot.z].downRamp())
        {
          xA = i;
          yA = j;
          endOfRound = false;
          i = 16;
          j = 16;
          decision = true;
        }
        else if(maze.robot.tile[i][j][maze.robot.z].downRamp() && !maze.robot.tile[i][j][maze.robot.z].upRamp())
        {
          xA = i;
          yA = j;
          endOfRound = false;
          i = 16;
          j = 16;
          decision = true;
        }
      }
    }

    if(maze.robot.z != zA && !decision)
    {
      if(maze.robot.z < zA)
      {
        for(int i = 0; i < 15; i++)
        {
          for(int j = 0; j < 15; j++)
          {
            if(maze.robot.tile[i][j][maze.robot.z].downRamp())
            {
              xA = i;
              yA = j;
              endOfRound = false;
              i = 16;
              j = 16;
            }
          }
        }
      }
      else if(maze.robot.z > zA)
      {
        for(int i = 0; i < 15; i++)
        {
          for(int j = 0; j < 15; j++)
          {
            if(maze.robot.tile[i][j][maze.robot.z].upRamp())
            {
              xA = i;
              yA = j;
              endOfRound = false;
              i = 16;
              j = 16;
            }
          }
        }
      }
    }

    //if(maze.robot.x==maze.robot.startX && maze.robot.y==maze.robot.startY  && maze.robot.z==maze.robot.startZ)
    //delay(30000);
    
  }
  }
  else
  {
    xA = xChina;
    yA = yChina;
  }

  //maze.sensors.motors.escribirNumLCD(xA);
  //delay(1000);
  //maze.sensors.motors.escribirNumLCD(yA);
  //delay(1000);

/*maze.sensors.motors.escribirNumLCD(xA);
 delay(500);
  maze.sensors.motors.escribirNumLCD(yA);
 delay(500);
 maze.sensors.motors.escribirNumLCD(zA);
 delay(500);*/

 if(xA == maze.robot.x && yA == maze.robot.y && zA == maze.robot.z)
 {
  endOfRound = true;
 }
 else if(xA == maze.robot.x && yA == maze.robot.y && zA != maze.robot.z)
 {
  maze.halfTurn();
  endOfRound = false;
 }
 else
 {

  maze.clear(xA, yA);
  maze.startBfs();
  maze.findWay();

  switch(maze.robot.getDirection())
    {
      case 'N':
        {
          maze.robot.x = xA+1;
          maze.robot.y = yA;
        } break;
      case 'E':
        {
          maze.robot.x = xA;
          maze.robot.y = yA-1;
        } break;
      case 'W':
        {
          maze.robot.x = xA;
          maze.robot.y = yA+1;
        } break;
      case 'S':
        {
          maze.robot.x = xA-1;
          maze.robot.y = yA;
        } break;
    }
 }
 
}
