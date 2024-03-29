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

void Priotities::changeSquare(Bit *actualSquare, float rightDistance, float leftDistance, float frontDistance, char dir)
{
  switch(dir)
    {
      case 'N':
        {
          (frontDistance > 20 || frontDistance == 0) ? actualSquare -> up(0, &tile[x-1][y][z]) : actualSquare -> up(1, &tile[x-1][y][z]);
          (rightDistance > 20 || rightDistance == 0) ? actualSquare -> right(0, &tile[x][y+1][z]) : actualSquare -> right(1, &tile[x][y+1][z]);
          (leftDistance > 20 || leftDistance == 0) ? actualSquare -> left(0, &tile[x][y-1][z]) : actualSquare -> left(1, &tile[x][y-1][z]);
        } break;
      case 'E':
        {
          (frontDistance > 20 || frontDistance == 0) ? actualSquare -> right(0, &tile[x][y+1][z]) : actualSquare -> right(1, &tile[x][y+1][z]);
          (rightDistance > 20 || rightDistance == 0) ? actualSquare -> down(0, &tile[x+1][y][z]) : actualSquare -> down(1, &tile[x+1][y][z]);
          (leftDistance > 20 || leftDistance == 0) ? actualSquare -> up(0, &tile[x-1][y][z]) : actualSquare -> up(1, &tile[x-1][y][z]);
        } break;
      case 'W':
        {
          (frontDistance > 20 || frontDistance == 0) ? actualSquare -> left(0, &tile[x][y-1][z]) : actualSquare -> left(1, &tile[x][y-1][z]);
          (rightDistance > 20 || rightDistance == 0) ? actualSquare -> up(0, &tile[x-1][y][z]) : actualSquare -> up(1, &tile[x-1][y][z]);
          (leftDistance > 20 || leftDistance == 0) ? actualSquare -> down(0, &tile[x+1][y][z]) : actualSquare -> down(1, &tile[x+1][y][z]);
        } break;
      case 'S':
        {
          (frontDistance > 20 || frontDistance == 0) ? actualSquare -> down(0, &tile[x+1][y][z]) : actualSquare -> down(1, &tile[x+1][y][z]);
          (rightDistance > 20 || rightDistance == 0) ? actualSquare -> left(0, &tile[x][y-1][z]) : actualSquare -> left(1, &tile[x][y-1][z]);
          (leftDistance > 20 || leftDistance == 0) ? actualSquare -> right(0, &tile[x][y+1][z]) : actualSquare -> right(1, &tile[x][y+1][z]);
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

  if(!actualSquare -> up() && !tile[x-1][y][z].visited() && !tile[x-1][y][z].pending())
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
  if(!actualSquare -> right() && !tile[x][y+1][z].visited() && !tile[x][y+1][z].pending())
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
  if(!actualSquare -> left() && !tile[x][y-1][z].visited() && !tile[x][y-1][z].pending())
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
  if(!actualSquare -> down() && !tile[x+1][y][z].visited() && !tile[x+1][y][z].pending())
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

 /* if(x == -1){
    this -> dataTransferUp();
    prueba = 1;}
  else if(x == 15){
    this -> dataTransferDown();
    prueba = 2;}
  else if(y == -1){
    this -> dataTransferLeft();
    prueba = 3;}
  else if(y == 15){
    this -> dataTransferRight();
    prueba = 4;}*/
}

void Priotities::correction(char dir)
{
  switch(dir)
  {
    case 'N':
    {
      tile[x][y][z].up(1, &tile[x-1][y][z]);
      tile[x-1][y][z].exist(0);
      tile[x-1][y][z].pending(0);
    } break;
    case 'E':
    {
      tile[x][y][z].right(1, &tile[x][y+1][z]);
      tile[x][y+1][z].exist(0);
      tile[x][y+1][z].pending(0);
    } break;
    case 'W':
    {
      tile[x][y][z].left(1, &tile[x][y-1][z]);
      tile[x][y-1][z].exist(0);
      tile[x][y-1][z].pending(0);
    } break;
    case 'S':
    {
      tile[x][y][z].down(1, &tile[x+1][y][z]);
      tile[x+1][y][z].exist(0);
      tile[x+1][y][z].pending(0);
    } break;
  }
}

void Priotities::dataTransferUp()
{
   for(int i = -1; i < 15; i++)
   {
     for(int j = -1; j < 15; j++)
     {
       tile[i+1][j][z].setInf1(tile[i][j][z].getInf1());
       tile[i+1][j][z].setInf2(tile[i][j][z].getInf2());
     }
   }

   x = 0;
   startX += 1;

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
  for(int i = 0; i < 16; i++)
   {
     for(int j = 0; j < 16; j++)
     {
       tile[i-1][j][z].setInf1(tile[i][j][z].getInf1());
       tile[i-1][j][z].setInf2(tile[i][j][z].getInf2());
     }
   }

   x = 14;
   startX -= 1;

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
  for(int i = 0; i < 16; i++)
   {
     for(int j = 0; j < 16; j++)
     {
       tile[i][j-1][z].setInf1(tile[i][j][z].getInf1());
       tile[i][j-1][z].setInf2(tile[i][j][z].getInf2());
     }
   }

   y = 14;
   startY -= 1;

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
  for(int i = -1; i < 15; i++)
   {
     for(int j = -1; j < 15; j++)
     {
       tile[i][j+1][z].setInf1(tile[i][j][z].getInf1());
       tile[i][j+1][z].setInf2(tile[i][j][z].getInf2());
     }
   }

   y = 0;
   startY += 1;

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

   for(int i = 0; i < 50; i++)
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

   for(int i = 0; i < 50; i++)
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
