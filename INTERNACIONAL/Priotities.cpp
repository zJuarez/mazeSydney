#include <Arduino.h>
#include "Priotities.h"

Priotities::Priotities(uint16_t _x, uint16_t _y, uint16_t _z, char _dir)
{
  x = _x;
  y = _y;
  z = _z;
  directi0n = _dir;
  startX = _x;
  startY = _y;
  startZ = _z;
}

void Priotities::changeDirection(uint8_t condition, char dir)
{
  switch(condition)
    {
      case 1:
      {
        switch(dir)
        {
          case 'N': directi0n = 'E';
          break;
          case 'E': directi0n = 'S';
          break;
          case 'W': directi0n = 'N';
          break;
          case 'S': directi0n = 'W';
          break;
        }
      } break;
      case 3:
      {
        switch(dir)
        {
          case 'N': directi0n = 'W';
          break;
          case 'E': directi0n = 'N';
          break;
          case 'W': directi0n = 'S';
          break;
          case 'S': directi0n = 'E';
          break;
        }
      } break;
    }
}

char Priotities::getDirection()
{
  return directi0n;
}

bool Priotities::correctMovementRight(char dir)
{    
      switch(dir)
        {
          case 'N':
            (tile[x][y][z].right() || tile[x][y+1][z].blackTile() || tile[x][y+1][z].visited() || !tile[x][y+1][z].exist()) ? returningVariable = false : returningVariable = true;
            break;
          case 'E':
            (tile[x][y][z].down() || tile[x+1][y][z].blackTile() || tile[x+1][y][z].visited() || !tile[x+1][y][z].exist()) ? returningVariable = false : returningVariable = true;
            break;
          case 'W':
            (tile[x][y][z].up() || tile[x-1][y][z].blackTile() || tile[x-1][y][z].visited() || !tile[x-1][y][z].exist()) ? returningVariable = false : returningVariable = true;
            break;
          case 'S':
            (tile[x][y][z].left() || tile[x][y-1][z].blackTile() || tile[x][y-1][z].visited() || !tile[x][y-1][z].exist()) ?  returningVariable = false : returningVariable = true;
            break;
        }

    return returningVariable;
}

bool Priotities::correctMovementLeft(char dir)
{
  switch(dir)
    {
      case 'N':
        (tile[x][y][z].left() || tile[x][y-1][z].blackTile() || tile[x][y-1][z].visited() || !tile[x][y-1][z].exist()) ? returningVariable = false : returningVariable = true;
        break;
      case 'E':
        (tile[x][y][z].up() || tile[x-1][y][z].blackTile() || tile[x-1][y][z].visited() || !tile[x-1][y][z].exist()) ? returningVariable = false : returningVariable = true;
        break;
      case 'W':
        (tile[x][y][z].down() || tile[x+1][y][z].blackTile() || tile[x+1][y][z].visited() || !tile[x+1][y][z].exist()) ? returningVariable = false : returningVariable = true;
        break;
      case 'S':
        (tile[x][y][z].right() || tile[x][y+1][z].blackTile() || tile[x][y+1][z].visited() || !tile[x][y+1][z].exist()) ? returningVariable = false : returningVariable = true;
        break;
    }

    return returningVariable;
}

bool Priotities::correctMovementFront(char dir)
{
  switch(dir)
    {
      case 'N':
      (tile[x][y][z].up() || tile[x-1][y][z].blackTile() || tile[x-1][y][z].visited() || !tile[x-1][y][z].exist()) ? returningVariable = false : returningVariable = true;
        break;
      case 'E':
      (tile[x][y][z].right() || tile[x][y+1][z].blackTile() || tile[x][y+1][z].visited() || !tile[x][y+1][z].exist()) ? returningVariable = false : returningVariable = true;
        break;
      case 'W':
      (tile[x][y][z].left() || tile[x][y-1][z].blackTile() || tile[x][y-1][z].visited() || !tile[x][y-1][z].exist()) ? returningVariable = false : returningVariable = true;
        break;
      case 'S':
      (tile[x][y][z].down() || tile[x+1][y][z].blackTile() || tile[x+1][y][z].visited() || !tile[x+1][y][z].exist()) ? returningVariable = false : returningVariable = true;
        break;
    }

    return returningVariable;
}

void Priotities::changeSquare(Bit *actualSquare, int rightDistance, int leftDistance, int frontDistance, char dir)
{
  switch(dir)
    {
      case 'N':
        {
          (frontDistance > 20) ? actualSquare -> up(0, &tile[x-1][y][z]) : actualSquare -> up(1, &tile[x-1][y][z]);
          (rightDistance > 20) ? actualSquare -> right(0, &tile[x][y+1][z]) : actualSquare -> right(1, &tile[x][y+1][z]);
          (leftDistance > 20) ? actualSquare -> left(0, &tile[x][y-1][z]) : actualSquare -> left(1, &tile[x][y-1][z]);
        } break;
      case 'E':
        {
          (frontDistance > 20) ? actualSquare -> right(0, &tile[x][y+1][z]) : actualSquare -> right(1, &tile[x][y+1][z]);
          (rightDistance > 20) ? actualSquare -> down(0, &tile[x+1][y][z]) : actualSquare -> down(1, &tile[x+1][y][z]);
          (leftDistance > 20) ? actualSquare -> up(0, &tile[x-1][y][z]) : actualSquare -> up(1, &tile[x-1][y][z]);
        } break;
      case 'W':
        {
          (frontDistance > 20) ? actualSquare -> left(0, &tile[x][y-1][z]) : actualSquare -> left(1, &tile[x][y-1][z]);
          (rightDistance > 20) ? actualSquare -> up(0, &tile[x-1][y][z]) : actualSquare -> up(1, &tile[x-1][y][z]);
          (leftDistance > 20) ? actualSquare -> down(0, &tile[x+1][y][z]) : actualSquare -> down(1, &tile[x+1][y][z]);
        } break;
      case 'S':
        {
          (frontDistance > 20) ? actualSquare -> down(0, &tile[x+1][y][z]) : actualSquare -> down(1, &tile[x+1][y][z]);
          (rightDistance > 20) ? actualSquare -> left(0, &tile[x][y-1][z]) : actualSquare -> left(1, &tile[x][y-1][z]);
          (leftDistance > 20) ? actualSquare -> right(0, &tile[x][y+1][z]) : actualSquare -> right(1, &tile[x][y+1][z]);
        } break;
    }
}

void Priotities::changeStatus(Bit *actualSquare, char dir)
{
  actualSquare -> visited(1);
  actualSquare -> pending(false);

  if(!actualSquare -> up())
    tile[x-1][y][z].exist(1);
  if(!actualSquare -> right())
    tile[x][y+1][z].exist(1);
  if(!actualSquare -> left())
    tile[x][y-1][z].exist(1);
  if(!actualSquare -> down())
    tile[x+1][y][z].exist(1);

  switch(dir)
  {
    case 'N':
    {
      if(!actualSquare -> right() && !tile[x][y+1][z].visited())
      {
        if(!actualSquare -> left() && !tile[x][y-1][z].visited())
        {
          tile[x][y-1][z].pending(1);

          if(z == 0)
          {            
            fFxs[firstCount] = x;
            fFys[firstCount] = y-1;

            firstCount++;
          }
          else if(z == 1)
          {
            sFxs[secondCount] = x;
            sFys[secondCount] = y-1;

            secondCount++;
          }
          else if(z == 2)
          {
            tFxs[thirdCount] = x;
            tFys[thirdCount] = y-1;

            thirdCount++;
          }
          else if(z == 3)
          {
            fhFxs[fourthCount] = x;
            fhFys[fourthCount] = y-1;

            fourthCount++;
          }
        }
        if(!actualSquare -> up() && !tile[x-1][y][z].visited())
        {
          tile[x-1][y][z].pending(1);

          if(z == 0)
          {            
            fFxs[firstCount] = x-1;
            fFys[firstCount] = y;

            firstCount++;
          }
          else if(z == 1)
          {
            sFxs[secondCount] = x-1;
            sFys[secondCount] = y;

            secondCount++;
          }
          else if(z == 2)
          {
            tFxs[thirdCount] = x-1;
            tFys[thirdCount] = y;

            thirdCount++;
          }
          else if(z == 3)
          {
            fhFxs[fourthCount] = x-1;
            fhFys[fourthCount] = y;

            fourthCount++;
          }
        }
      }
      else if(!actualSquare -> up() && !tile[x-1][y][z].visited())
      {
        if(!actualSquare -> left() && !tile[x][y-1][z].visited())
        {
          tile[x][y-1][z].pending(1);

          if(z == 0)
          {            
            fFxs[firstCount] = x;
            fFys[firstCount] = y-1;

            firstCount++;
          }
          else if(z == 1)
          {
            sFxs[secondCount] = x;
            sFys[secondCount] = y-1;

            secondCount++;
          }
          else if(z == 2)
          {
            tFxs[thirdCount] = x;
            tFys[thirdCount] = y-1;

            thirdCount++;
          }
          else if(z == 3)
          {
            fhFxs[fourthCount] = x;
            fhFys[fourthCount] = y-1;

            fourthCount++;
          }
        }
      }
    } break;
    case 'E':
    {
      if(!actualSquare -> down() && !tile[x+1][y][z].visited())
      {
        if(!actualSquare -> up() && !tile[x-1][y][z].visited())
        {
          tile[x-1][y][z].pending(1);

          if(z == 0)
          {            
            fFxs[firstCount] = x-1;
            fFys[firstCount] = y;

            firstCount++;
          }
          else if(z == 1)
          {
            sFxs[secondCount] = x-1;
            sFys[secondCount] = y;

            secondCount++;
          }
          else if(z == 2)
          {
            tFxs[thirdCount] = x-1;
            tFys[thirdCount] = y;

            thirdCount++;
          }
          else if(z == 3)
          {
            fhFxs[fourthCount] = x-1;
            fhFys[fourthCount] = y;

            fourthCount++;
          }
        }
        if(!actualSquare -> right() && !tile[x][y+1][z].visited())
        {
          tile[x][y+1][z].pending(1);

          if(z == 0)
          {            
            fFxs[firstCount] = x;
            fFys[firstCount] = y+1;

            firstCount++;
          }
          else if(z == 1)
          {
            sFxs[secondCount] = x;
            sFys[secondCount] = y+1;

            secondCount++;
          }
          else if(z == 2)
          {
            tFxs[thirdCount] = x;
            tFys[thirdCount] = y+1;

            thirdCount++;
          }
          else if(z == 3)
          {
            fhFxs[fourthCount] = x;
            fhFys[fourthCount] = y+1;

            fourthCount++;
          }
        }
      }
      else if(!actualSquare -> right() && !tile[x][y+1][z].visited())
      {
        if(!actualSquare -> up() && !tile[x-1][y][z].visited())
        {
          tile[x-1][y][z].pending(1);

          if(z == 0)
          {            
            fFxs[firstCount] = x-1;
            fFys[firstCount] = y;

            firstCount++;
          }
          else if(z == 1)
          {
            sFxs[secondCount] = x-1;
            sFys[secondCount] = y;

            secondCount++;
          }
          else if(z == 2)
          {
            tFxs[thirdCount] = x-1;
            tFys[thirdCount] = y;

            thirdCount++;
          }
          else if(z == 3)
          {
            fhFxs[fourthCount] = x-1;
            fhFys[fourthCount] = y;

            fourthCount++;
          }
        }
      }
    } break;
    case 'W':
    {
      if(!actualSquare -> up() && !tile[x-1][y][z].visited())
      {
        if(!actualSquare -> down() && !tile[x+1][y][z].visited())
        {
          tile[x+1][y][z].pending(1);

          if(z == 0)
          {            
            fFxs[firstCount] = x+1;
            fFys[firstCount] = y;

            firstCount++;
          }
          else if(z == 1)
          {
            sFxs[secondCount] = x+1;
            sFys[secondCount] = y;

            secondCount++;
          }
          else if(z == 2)
          {
            tFxs[thirdCount] = x+1;
            tFys[thirdCount] = y;

            thirdCount++;
          }
          else if(z == 3)
          {
            fhFxs[fourthCount] = x+1;
            fhFys[fourthCount] = y;

            fourthCount++;
          }
        }
        if(!actualSquare -> left() && !tile[x][y-1][z].visited())
        {
          tile[x][y-1][z].pending(1);

          if(z == 0)
          {            
            fFxs[firstCount] = x;
            fFys[firstCount] = y-1;

            firstCount++;
          }
          else if(z == 1)
          {
            sFxs[secondCount] = x;
            sFys[secondCount] = y-1;

            secondCount++;
          }
          else if(z == 2)
          {
            tFxs[thirdCount] = x;
            tFys[thirdCount] = y-1;

            thirdCount++;
          }
          else if(z == 3)
          {
            fhFxs[fourthCount] = x;
            fhFys[fourthCount] = y-1;

            fourthCount++;
          }
        }
      }
      else if(!actualSquare -> left() && !tile[x][y-1][z].visited())
      {
        if(!actualSquare -> down() && !tile[x+1][y][z].visited())
        {
          tile[x+1][y][z].pending(1);

          if(z == 0)
          {            
            fFxs[firstCount] = x+1;
            fFys[firstCount] = y;

            firstCount++;
          }
          else if(z == 1)
          {
            sFxs[secondCount] = x+1;
            sFys[secondCount] = y;

            secondCount++;
          }
          else if(z == 2)
          {
            tFxs[thirdCount] = x+1;
            tFys[thirdCount] = y;

            thirdCount++;
          }
          else if(z == 3)
          {
            fhFxs[fourthCount] = x+1;
            fhFys[fourthCount] = y;

            fourthCount++;
          }
        }
      }
    } break;
    case 'S':
    {
      if(!actualSquare -> left() && !tile[x][y-1][z].visited())
      {
        if(!actualSquare -> right() && !tile[x][y+1][z].visited())
        {
          tile[x][y+1][z].pending(1);

          if(z == 0)
          {            
            fFxs[firstCount] = x;
            fFys[firstCount] = y+1;

            firstCount++;
          }
          else if(z == 1)
          {
            sFxs[secondCount] = x;
            sFys[secondCount] = y+1;

            secondCount++;
          }
          else if(z == 2)
          {
            tFxs[thirdCount] = x;
            tFys[thirdCount] = y+1;

            thirdCount++;
          }
          else if(z == 3)
          {
            fhFxs[fourthCount] = x;
            fhFys[fourthCount] = y+1;

            fourthCount++;
          }
        }
        if(!actualSquare -> down() && !tile[x+1][y][z].visited())
        {
          tile[x+1][y][z].pending(1);

          if(z == 0)
          {            
            fFxs[firstCount] = x+1;
            fFys[firstCount] = y;

            firstCount++;
          }
          else if(z == 1)
          {
            sFxs[secondCount] = x+1;
            sFys[secondCount] = y;

            secondCount++;
          }
          else if(z == 2)
          {
            tFxs[thirdCount] = x+1;
            tFys[thirdCount] = y;

            thirdCount++;
          }
          else if(z == 3)
          {
            fhFxs[fourthCount] = x+1;
            fhFys[fourthCount] = y;

            fourthCount++;
          }
        }
      }
      else if(!actualSquare -> down() && !tile[x+1][y][z].visited())
      {
        if(!actualSquare -> right() && !tile[x][y+1][z].visited())
        {
          tile[x][y+1][z].pending(1);

          if(z == 0)
          {            
            fFxs[firstCount] = x;
            fFys[firstCount] = y+1;

            firstCount++;
          }
          else if(z == 1)
          {
            sFxs[secondCount] = x;
            sFys[secondCount] = y+1;

            secondCount++;
          }
          else if(z == 2)
          {
            tFxs[thirdCount] = x;
            tFys[thirdCount] = y+1;

            thirdCount++;
          }
          else if(z == 3)
          {
            fhFxs[fourthCount] = x;
            fhFys[fourthCount] = y+1;

            fourthCount++;
          }
        }
      }
    } break;
  }
}

void Priotities::changeCoordinates(char dir)
{
  switch(dir)
    {
      case 'N': x--;
      break;
      case 'E': y++;
      break;
      case 'W': y--;
      break;
      case 'S': x++;
      break;
    }

 /* if(x == -1)
    this -> dataTransferUp();
  else if(x == 15)
    this -> dataTransferDown();
  else if(y == -1)
    this -> dataTransferLeft();
  else if(y == 15)
    this -> dataTransferRight();*/
}

void Priotities::correction(char dir)
{
  switch(dir)
  {
    case 'N':
    {
      tile[x][y][z].up(1, &tile[x-1][y][z]);

      if(tile[x-1][y][z].right() == false || tile[x-1][y][z].left() == false || tile[x-1][y][z].down() == false){}
      else{
      tile[x-1][y][z].exist(0);
      tile[x-1][y][z].pending(0);}
    } break;
    case 'E':
    {
      tile[x][y][z].right(1, &tile[x][y+1][z]);
      
      if(tile[x][y+1][z].right() == false || tile[x][y+1][z].left() == false || tile[x][y+1][z].down() == false){}
      else{
      tile[x][y+1][z].exist(0);
      tile[x][y+1][z].pending(0);}
    } break;
    case 'W':
    {
      tile[x][y][z].left(1, &tile[x][y-1][z]);
      
      if(tile[x][y-1][z].right() == false || tile[x][y-1][z].left() == false || tile[x][y-1][z].down() == false){}
      else{
      tile[x][y-1][z].exist(0);
      tile[x][y-1][z].pending(0);}
    } break;
    case 'S':
    {
      tile[x][y][z].down(1, &tile[x+1][y][z]);
      
      if(tile[x+1][y][z].right() == false || tile[x+1][y][z].left() == false || tile[x+1][y][z].down() == false){}
      else{
      tile[x+1][y][z].exist(0);
      tile[x+1][y][z].pending(0);}
    } break;
  }
}

void Priotities::dataTransferUp()
{
   for(int i = 14; i >= 0; i--)
   {
     for(int j = 14; j >= 0; j--)
     {
      if(i+1 != 15)
      {
        tile[i+1][j][z].setInf1(tile[x][y][z].getInf1());
        tile[i+1][j][z].setInf2(tile[x][y][z].getInf2());
      }
      if(i == 0)
      {
        tile[i][j][z].exist(NULL);
        tile[i][j][z].pending(NULL);
        tile[i][j][z].down(NULL, NULL);
        tile[i][j][z].up(NULL, NULL);
        tile[i][j][z].left(NULL, NULL);
        tile[i][j][z].right(NULL, NULL);
        tile[i][j][z].blackTile(NULL);
        tile[i][j][z].checkpoint(NULL);
        tile[i][j][z].victim(NULL);
        tile[i][j][z].visited(NULL);
        tile[i][j][z].flo0r(NULL);
        tile[i][j][z].ramp(NULL);
        tile[i][j][z].start(NULL);
        tile[i][j][z].bumper(NULL);      
      }
     }
   }
   
   x = 0;
   startX++;

   if(z == 0)
   {
     for(int i = 0; i < firstCount; i++)
      fFxs[i]++;
   }
   else if(z == 1)
   {
     for(int i = 0; i < secondCount; i++)
      sFxs[i]++;
   }
   else if(z == 2)
   {
     for(int i = 0; i < thirdCount; i++)
      tFxs[i]++;
   }
   else if(z == 3)
   {
     for(int i = 0; i < fourthCount; i++)
      fhFxs[i]++;
   }
}

void Priotities::dataTransferDown()
{
  for(int i = 0; i < 15; i++)
   {
     for(int j = 0; j < 15; j++)
     {
      if(i-1 != -1)
      {
       tile[i-1][j][z].setInf1(tile[i][j][z].getInf1());
       tile[i-1][j][z].setInf2(tile[i][j][z].getInf2());
      }
      if(i == 14)
      {
        tile[i][j][z].exist(NULL);
        tile[i][j][z].pending(NULL);
        tile[i][j][z].down(NULL, NULL);
        tile[i][j][z].up(NULL, NULL);
        tile[i][j][z].left(NULL, NULL);
        tile[i][j][z].right(NULL, NULL);
        tile[i][j][z].blackTile(NULL);
        tile[i][j][z].checkpoint(NULL);
        tile[i][j][z].victim(NULL);
        tile[i][j][z].visited(NULL);
        tile[i][j][z].flo0r(NULL);
        tile[i][j][z].ramp(NULL);
        tile[i][j][z].start(NULL);
        tile[i][j][z].bumper(NULL);    
      }
     }
   }

   x = 14;
   startX--;

   if(z == 0)
   {
     for(int i = 0; i < firstCount; i++)
      fFxs[i]--;
   }
   else if(z == 1)
   {
     for(int i = 0; i < secondCount; i++)
      sFxs[i]--;
   }
   else if(z == 2)
   {
     for(int i = 0; i < thirdCount; i++)
      tFxs[i]--;
   }
   else if(z == 3)
   {
     for(int i = 0; i < fourthCount; i++)
      fhFxs[i]--;
   }
}

void Priotities::dataTransferRight()
{
  for(int i = 0; i < 15; i++)
   {
     for(int j = 0; j < 15; j++)
     {
      if(j-1 != -1)
      {
        tile[i][j-1][z].setInf1(tile[i][j][z].getInf1());
        tile[i][j-1][z].setInf2(tile[i][j][z].getInf2());
      }
      if(j == 14)
      {
        tile[i][j][z].exist(NULL);
        tile[i][j][z].pending(NULL);
        tile[i][j][z].down(NULL, NULL);
        tile[i][j][z].up(NULL, NULL);
        tile[i][j][z].left(NULL, NULL);
        tile[i][j][z].right(NULL, NULL);
        tile[i][j][z].blackTile(NULL);
        tile[i][j][z].checkpoint(NULL);
        tile[i][j][z].victim(NULL);
        tile[i][j][z].visited(NULL);
        tile[i][j][z].flo0r(NULL);
        tile[i][j][z].ramp(NULL);
        tile[i][j][z].start(NULL);
        tile[i][j][z].bumper(NULL);
      }
     }
   }

   y = 14;
   startY--;

   if(z == 0)
   {
     for(int i = 0; i < firstCount; i++)
      fFys[i]--;
   }
   else if(z == 1)
   {
     for(int i = 0; i < secondCount; i++)
      sFys[i]--;
   }
   else if(z == 2)
   {
     for(int i = 0; i < thirdCount; i++)
      tFys[i]--;
   }
   else if(z == 3)
   {
     for(int i = 0; i < fourthCount; i++)
      fhFys[i]--;
   }
}

void Priotities::dataTransferLeft()
{
  for(int i = 14; i >= 0; i--)
   {
     for(int j = 14; j >= 0; j--)
     {
      if(j+1 != 15)
      {
        tile[i][j+1][z].setInf1(tile[x][y][z].getInf1());
        tile[i][j+1][z].setInf2(tile[x][y][z].getInf2());
      }
      if(j == 0)
      {
        tile[i][j][z].exist(NULL);
        tile[i][j][z].pending(NULL);
        tile[i][j][z].down(NULL, NULL);
        tile[i][j][z].up(NULL, NULL);
        tile[i][j][z].left(NULL, NULL);
        tile[i][j][z].right(NULL, NULL);
        tile[i][j][z].blackTile(NULL);
        tile[i][j][z].checkpoint(NULL);
        tile[i][j][z].victim(NULL);
        tile[i][j][z].visited(NULL);
        tile[i][j][z].flo0r(NULL);
        tile[i][j][z].ramp(NULL);
        tile[i][j][z].start(NULL);
        tile[i][j][z].bumper(NULL);      
      }
     }
   }

   y = 0;
   startY ++;

   if(z == 0)
   {
     for(int i = 0; i < firstCount; i++)
      fFys[i]++;
   }
   else if(z == 1)
   {
     for(int i = 0; i < secondCount; i++)
      sFys[i]++;
   }
   else if(z == 2)
   {
     for(int i = 0; i < thirdCount; i++)
      tFys[i]++;
   }
   else if(z == 3)
   {
     for(int i = 0; i < fourthCount; i++)
      fhFys[i]++;
   }
}

void Priotities::dataTransferUpFloor()
{
  for(int i = 0; i < 15; i++)
   {
     for(int j = 0; j < 15; j++)
     {
       for(int k = 0; k < 5; z++)
       {
         tile[i][j][k-1].setInf1(tile[i][j][k].getInf1());
         tile[i][j][k-1].setInf2(tile[i][j][k].getInf2());
       }
     }
   }

   z = 3;
   startZ--;

   for(int i = 0; i < 100; i++)
   {
     fFxs[i] = sFxs[i];
     sFxs[i] = tFxs[i];
     tFxs[i] = fhFxs[i];
   }

   firstCount = secondCount;
   secondCount = thirdCount;
   thirdCount = fourthCount;
   fourthCount = 0;
}

void Priotities::dataTransferDownFloor()
{
  for(int i = 0; i < 15; i++)
   {
     for(int j = 0; j < 15; j++)
     {
       for(int k = -1; k < 4; z++)
       {
         tile[i][j][k+1].setInf1(tile[i][j][k].getInf1());
         tile[i][j][k+1].setInf2(tile[i][j][k].getInf2());
       }
     }
   }

   z = 0;
   startZ++;

   for(int i = 0; i < 100; i++)
   {
     sFxs[i] = fFxs[i];
     tFxs[i] = sFxs[i];
     fhFxs[i] = tFxs[i];
   }

   secondCount = firstCount;
   thirdCount = secondCount;
   fourthCount = thirdCount;
   firstCount = 0;
}
