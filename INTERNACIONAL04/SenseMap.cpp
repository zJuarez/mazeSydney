#include <SingleEMAFilterLib.h>
 #include <Arduino.h>
#include "SenseMap.h"

Ultrasonic front(28, 29);
Ultrasonic raight(30, 31);
Ultrasonic left(32, 33);
Ultrasonic back(35, 34);
SingleEMAFilter<int> frontF(0.77);
SingleEMAFilter<int> leftF(0.77);
SingleEMAFilter<int> backF(0.77);
SingleEMAFilter<int> raightF(0.77);

SenseMap::SenseMap(){}

void SenseMap::setup()
{
  pinMode(37, OUTPUT);
  pinMode(38, OUTPUT);
  pinMode(39, OUTPUT);
}

int SenseMap::getDistanceOf(uint8_t num)
{  
  if(num == 1){
    distance = left.read();
    return leftF.AddValue(distance);
  }
  else if(num == 2){
    distance = raight.read();
    return raightF.AddValue(distance);
  }
  else if(num == 3){
    distance = front.read();
    return frontF.AddValue(distance);
  }
  else if(num == 4){
    distance = back.read();
    return backF.AddValue(distance);
  }
  return distance;
}

void SenseMap::checkDistances(bool alg)
{
motors.setBase(motors.velInicial);

int d1 = this -> getDistanceOf(3);
int pp;

if (motors.bumperControl)
pp=18;
else
pp=14;


  if(d1 >= 7 && d1 <= pp)
  {  
    unsigned long tempoo = millis();
    
    while(this -> getDistanceOf(3) >= 7.3)
    {
      motors.avanzar(motors.de,30,30,motors.bD);
      if(!motors.banderaTotalCalor && !alg)
      {
        if(motors.heatVictimC(0))
        tempoo=millis();
      }
      if(millis()>(tempoo+400))
        break;
    }
    
  motors.detenerse();
  }
  else if(d1 <= 6 || d1>220)
  {
    unsigned long ter = millis();
    bool ban = false;
    
    if(d1==357)
    {
    motors.atrasPID(motors.de);
    delay(130);
    motors.detenerse();    
    ban=true;
    }
    
   int newD = this -> getDistanceOf(3);
   
    while((newD <= 7 || newD>220) && !ban)
    {
      
      motors.atrasPID(motors.de);
      
      if(millis()>(ter+300))
      break;

      newD = this -> getDistanceOf(3);
    }
    motors.detenerse();
  }
  
  int disss = this -> getDistanceOf(3);
  
  if(disss >= 25 && disss <= 45) //36
  {
    unsigned long terr = millis();
    
    if(disss >= 32 && disss <= 38)
    {}
    else if(disss > 38)
    {
      while(this -> getDistanceOf(3) > 38)
      {
        motors.avanzar(motors.de,30,30,motors.bD);
        
      if(!motors.banderaTotalCalor && !alg)
      {
        if(motors.heatVictimC(0))
        terr=millis();
      }
        if(millis() > (terr + 300))
          break;
      }
      
     motors.detenerse();
    }
    else if(disss < 31)
    {
      while(this -> getDistanceOf(3) < 31)
       {
          motors.atrasPID(motors.de);
          
      if(!motors.banderaTotalCalor && !alg)
      {
        if(motors.heatVictimC(0))
        terr=millis();
      }
      
          if(millis() > (terr+300))
           break;
       }
       
     motors.detenerse();
    }
    
  }
  else if(disss >= 55 && disss <= 75) //36
  {

    unsigned long terr2 = millis();
    
    if(disss >= 61 && disss <= 67)
    {}
    else if(disss > 67)
    {
      while(this -> getDistanceOf(3) > 67)
      {
        motors.avanzar(motors.de,30,30,motors.bD);

      if(!motors.banderaTotalCalor && !alg)
      {
        if(motors.heatVictimC(0))
        terr2=millis();
      }
      
        if(millis() > (terr2 + 300))
          break;
      }
      
     motors.detenerse();
    }
    else if(disss < 61)
    {
      while(this -> getDistanceOf(3) < 61)
       {
          motors.atrasPID(motors.de);

      if(!motors.banderaTotalCalor && !alg)
      {
        if(motors.heatVictimC(0))
        terr2=millis();
      }
      
          if(millis() > (terr2+300))
           break;
       }
       
     motors.detenerse();
    }
    
  }

int dAtras = this -> getDistanceOf(4);
if(dAtras>=7 && dAtras<=13)
{
  unsigned long terr22= millis();
  
  while(dAtras>=7 && dAtras<=13){
  motors.atrasPID(motors.de);
 if(millis() > (terr22+240))
    break;
   dAtras = this -> getDistanceOf(4);
  }

       motors.detenerse();
}
}

void SenseMap::acomodo(double dEI,double dAI)
{
  if(abs(dEI-dAI)<20)
  dAI=300;
  
  char pos = dEI<dAI?'E':'A';
  double m = dEI<=dAI?dEI:dAI;
  
  if(m>90)
  return;
  
unsigned long nlp = millis();

int k = teMamaste(m,pos);
  
    if(k!=0){
    motors.setBase(motors.velInicial);

    while(k!=0)
    {
      k=teMamaste(m,pos);
      
      if(k==1)
      motors.avanzar(motors.de,30,30,motors.bD);
      else if(k==2)
      {
   motors.detenerse();
   return;
      }

         if(!motors.banderaTotalCalor)
      {
        if(motors.heatVictimC(0))
        nlp=millis();
      }
      
      if(millis()>(nlp+650))
      {
   motors.detenerse();
   return;
      }
    }
 motors.detenerse();
   }
}
int SenseMap::teMamaste(double disI, char pos)
{
  double dActual;
  if(pos=='E')
  {
    dActual=this -> getDistanceOf(3);
    
    if(dActual<(disI-33))
    return 1;
    else if(dActual>(disI-27))
    return 2;
    else
    return 0;
  }
  else if (pos == 'A')
  {
    dActual=this -> getDistanceOf(4);

    
    if(dActual<(disI+27))
    return 1;
    else if(dActual>(disI+33))
    return 2;
    else 
    return 0;
  }
}
