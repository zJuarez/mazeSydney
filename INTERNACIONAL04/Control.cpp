#include "Control.h"


unsigned long pasado=0;
byte TiempoMuestreo=1;
double setPointBNO=0;
double setPointY=0;
int b=4;
int valorT;
int bD1=0;

//derecho
double base = 200;
double kp=5.43;//2.23
double ki=0.15;//0.08;
double kd=0.56;//0.56;
long int errorPass=0;
double errorD=0;
double errorAnt=0;
double error=0;
int limit = 65;



//giros
double kpGiro=2.65; //1.2  , 2.37 , 3.45
double kiGiro=0.31;  //0.6
double kdGiro=0.23;  //0.47
long int errorPassG=0;
double errorDG=0;
double errorAntG=0;
double errorG=0;
double pwmGiroT=100;
bool right=false;



LiquidCrystal_I2C lcd(0x3F,16,2);
VL53L0X sensor;

//#define LONG_RANGE



//#define HIGH_SPEED


  #define HIGH_ACCURACY

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X);


bool Control::desalineao(double anguloA, double deseado)
{
  if(deseado==0)
  {
    double nA = anguloA>=180?anguloA:(360+anguloA);

    if(nA>(360-b) && nA<(360+b))
      return 0;
   else
    return 1;
  }
  else {

if(anguloA>(deseado-b) && anguloA<(deseado+b))
  return 0;
else
  return 1;

}

}

Control::Control(){
bno = Adafruit_BNO055();
}
int Control::orientationStatus()
{

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  //Serial.print("CALIBRATION: Sys=");
  //Serial.print(system, DEC);
  //Serial.print(" Gyro=");
  //Serial.print(gyro, DEC);
  //Serial.print(" Accel=");
  //Serial.print(accel, DEC);
  //Serial.print(" Mag=");
  //Serial.println(mag, DEC);

  //delay(90);

  return mag;
}

double Control::getAnguloActual()
{
double anguloA=0;
sensors_event_t event;
bno.getEvent(&event);
anguloA = event.orientation.x;
anguloA-=setPointBNO;

if(anguloA<0)
anguloA+=360;

if(anguloA>360)
anguloA-=360;

return anguloA;
}

void Control::actualizaSetPoint(double d)
{
double a =getAnguloActual();
if(d==0 && a>=180)
{
setPointBNO=setPointBNO-(360-a);
}
else
setPointBNO=setPointBNO+(a-d);

setPointBNO=setPointBNO>360? (setPointBNO-360): setPointBNO;
setPointBNO=setPointBNO<-360?(setPointBNO+360): setPointBNO;
}

void Control::escribirLCDabajo(String sE1) {
  lcd.setCursor(0, 1);
  lcd.print(sE1);
}

void Control::escribirLCD(String sE1, String sE2) {
  lcd.clear();
  lcd.print(sE1);
  lcd.setCursor(0, 1);
  lcd.print(sE2);
}

void Control::setBase(double x)
{
base=x;
}

double Control::y()
{
  double ye;
  sensors_event_t event;
  bno.getEvent(&event);
  ye = event.orientation.z;
  ye-=setPointY;
  return ye;
}

bool Control::condRampa(bool s)
{
  if(s){
  if(y()>-3)
  { 
    limit=65;
    return false;
  }
  else 
  return true;
}
else
{
  if(y()>-3)
  return false;
  else
  return true;
}
}

int Control::rampa(double d)
{

if(y()<-320 && y()>-348) //335
{
  
base=200; //150  y 100
limit=100;
/*
lcd.setCursor(0,0);
lcd.print("Subiendo");
while(y()<-0.5){
avanzar(d,30, 30,bD1);
}
base=155;
lcd.clear();
avanzar(d,30,30,bD1);
delay(265);
detenerse();
//delay(155);
*/
  /*sensors_event_t event;
  bno.getEvent(&event);
  setPointY=event.orientation.z;*/
  
bumperControl=false;
return 1;
}
else if(y()<-4.8 && y()>-35)
{
base=100;
/*
lcd.setCursor(0,0);
lcd.print("Bajando");
while(y()<-0.4){
avanzar(d,30,30,bD1);
}
base=155;
lcd.clear();
avanzar(d,30,30,bD1);
delay(170);
detenerse();
//delay(300);
*/
/*sensors_event_t event;
bno.getEvent(&event);
setPointY=event.orientation.z;*/
bumperControl=false;
return 2;
}
else
return 0;

}

bool Control::bumper(uint8_t &x)
{
double val = y();
if((val<-358 && val>-340) || (val<-2.7 && val>-15))
{
  
if((val<-358 && val>-355) || (val<-2 && val>-5))
x=1;
else if((val<-355 && val>-351) || (val<-5 && val>-9))
x=2;
else if((val<-351 && val>-340) || (val<-9 && val>-15))
x=3;

return true;
}
else
{
x=0;
return false;
}
}

void Control::giroIzq(double deseado)
{
double anguloA=0;
anguloA = getAnguloActual();

double errorG=0;
unsigned long ahora=millis();
unsigned long CambioTiempo= ahora-pasado;

if(CambioTiempo >= TiempoMuestreo){

     if(anguloA<deseado)
    {
      errorG=360-deseado+anguloA;
    }
    else
    {
    if(deseado!=0)
    errorG=anguloA-deseado;
    else
    errorG=anguloA;
    }

    errorPassG= errorPassG*2/3.0+errorG*TiempoMuestreo;
    errorDG=(errorG-errorAntG)/TiempoMuestreo;

float P;
float I;
float D;
P=kpGiro*errorG;
I=kiGiro*errorPassG;
D=kdGiro*errorDG;

pwmGiroT=P+I+D;

pasado=ahora;
errorAntG=errorG;

}

if(pwmGiroT>255)
  pwmGiroT=255;

if(pwmGiroT<145)
 pwmGiroT=145;

    digitalWrite(motorIzqAde1, LOW);
    analogWrite(motorIzqAde2, pwmGiroT);
    digitalWrite(motorIzqAtras2, LOW);
    analogWrite(motorIzqAtras1, pwmGiroT);
    digitalWrite(motorDerAde1, LOW);
    analogWrite(motorDerAde2, pwmGiroT);
    digitalWrite(motorDerAtras1, LOW);
    analogWrite(motorDerAtras2, pwmGiroT);
}

void Control::giroDer(double deseado)
{
double anguloA=0;
anguloA = getAnguloActual();

unsigned long ahora=millis();
unsigned long CambioTiempo= ahora-pasado;

if(CambioTiempo >= TiempoMuestreo){

 if(anguloA>deseado)
    {
      errorG=deseado+(360-anguloA);
    }
    else
    {
    if(deseado!=0)
    errorG=deseado-anguloA;
    else
    errorG=360-anguloA;
    }

    errorPassG= errorPassG*2/3.0+errorG*TiempoMuestreo;
    errorDG=(errorG-errorAntG)/TiempoMuestreo;


float P;
float I;
float D;
P=kpGiro*errorG;
I=kiGiro*errorPassG;
D=kdGiro*errorDG;

pwmGiroT=P+I+D;

pasado=ahora;
errorAntG=errorG;
}

  if(pwmGiroT>255)
  pwmGiroT=255;
  if(pwmGiroT<145)
  pwmGiroT=145;

    digitalWrite(motorIzqAde2, LOW);
    analogWrite(motorIzqAde1, pwmGiroT);
    digitalWrite(motorIzqAtras1, LOW);
    analogWrite(motorIzqAtras2, pwmGiroT);
    digitalWrite(motorDerAde2, LOW);
    analogWrite(motorDerAde1, pwmGiroT);
    digitalWrite(motorDerAtras2, LOW);
    analogWrite(motorDerAtras1, pwmGiroT);
}
void Control::checa(double deseado)
{
double angulo=getAnguloActual();
unsigned long ahora=millis();
unsigned long tiempoT=millis();
int v=90;
b=2;

if(desalineao(angulo,deseado)){
while(desalineao(angulo,deseado))
{
angulo=getAnguloActual();

if(millis()>(ahora+700))
{
v+=40;
v=v>=255?255:v;
ahora=millis();
}

dondeGirar1(deseado,v);

if(millis()>(tiempoT+5000))
return;

}
detenerse();
delay(50);
}
b=4;
}

void Control::dondeGirar1(double d,int v)
{
double a = getAnguloActual();
double equis =0;
if (d > a){
    equis = d - a;
    if(equis<180){
giroD(v);
right=true;
}
    else{
giroI(v);
right=false;
}
  }
else{
    equis = a-d;

    if (equis<180){
     giroI(v);
     right=false;
}
    else{
    giroD(v);
 right=true;
}
    }

}

void Control::giroD(int v)
{
digitalWrite(motorIzqAde2, LOW);
    analogWrite(motorIzqAde1, v);
    digitalWrite(motorIzqAtras1, LOW);
    analogWrite(motorIzqAtras2, v);
    digitalWrite(motorDerAde2, LOW);
    analogWrite(motorDerAde1, v);
    digitalWrite(motorDerAtras2, LOW);
    analogWrite(motorDerAtras1, v);

}
void Control::giroI(int v)
{
    digitalWrite(motorIzqAde1, LOW);
    analogWrite(motorIzqAde2, v);
    digitalWrite(motorIzqAtras2, LOW);
    analogWrite(motorIzqAtras1, v);
    digitalWrite(motorDerAde1, LOW);
    analogWrite(motorDerAde2, v);
    digitalWrite(motorDerAtras1, LOW);
    analogWrite(motorDerAtras2, v);

}
bool Control::heatVictimC(bool bGiro)
{
  if(temperatureCelcius(mlxLeft) >= 25)
  {
    detenerse();
    blinkLeds();
    banderaTotalCalor=true;
    double newD;

    if(!bGiro)
    {
     newD = de==270?0:(de+90);
        giro(newD,1);
        detenerse();
        delay(150);
        dispenser.dropOneKitLeft();
        giro(de,1);
        detenerse();
        delay(150);
        checar(); 
        return true;
    }
    else
    {
    double a = getAnguloActual();
    newD = a +90;
    if (newD>=360)
    newD=newD-360;
    giro(newD,1);
    detenerse();
    delay(150);
      dispenser.dropOneKitLeft();
      return true;
    }
  }
  else if (temperatureCelcius(mlxRight) >= 25)
  {
    detenerse();
    blinkLeds();
    banderaTotalCalor=true;
    double newD;
    
    if(!bGiro){
      newD = de==0?270:(de-90);
        giro(newD,1);
        detenerse();
        delay(150);
        dispenser.dropOneKitRight();
        giro(de,1);
        detenerse();
        delay(150);
        checar(); 
        return true;
    }
    else{
      double a = getAnguloActual();
      newD = a-90;
      if(newD<0)
      newD=360+newD;

      giro(newD,1);
      detenerse();
      delay(150);
      dispenser.dropOneKitRight();
      return true;
    }
        
  }

  return false;
}
void Control::giro(double deseado, bool alg)
{
double angulo=getAnguloActual();
unsigned long nepe = millis();
uint8_t t=0;
while(desalineao(angulo,deseado))
{

if(bumper(t))
{
bumperControl=true;
}

if(!banderaTotalCalor && !alg)
{
  if(heatVictimC(1))
  nepe=millis();
}

angulo=getAnguloActual();

if(millis()<(nepe+3500))
dondeGirar(deseado);
else if(millis()<(nepe+5500)){
dondeGirar1(deseado,255);
bumperControl=true;
}
else
{
unsigned long jojo = millis();
if(right){
  int contad = 0;
while(desalineao(angulo,deseado))
{
angulo=getAnguloActual();

if(millis()<(jojo+3000))
giroIzq(deseado);
else if(millis()<(jojo+5000))
giroI(255);
else{
if(contad==0){
moveAtras();
delay(180);
contad++;
}
else if(contad==1){
moveAdelanteFast();
delay(190);
contad++;
}
else if(contad==2)
{desacomodo(100);
 contad++;
}
else if(contad==3)
{
desacomodo(200);
contad++;
}
else if(contad==4)
return;

jojo=millis();
}
}
return;
}
else
{
  int contadore = 0;
  
while(desalineao(angulo,deseado))
{
angulo=getAnguloActual();

if(millis()<(jojo+3000))
giroDer(deseado);
else if(millis()<(jojo+6000))
giroD(255);
else{
if(contadore ==0){
moveAtras();
delay(180);
contadore++;
}
else if(contadore==1)
{
  moveAdelanteFast();
  delay(200);
  contadore++;
}
else if(contadore==2)
{
  desacomodo(100);
  contadore++;
}
else if(contadore==3)
{
  desacomodo(200);
  contadore++;
}
else if(contadore==4)
return;

jojo=millis();
}
}
return;
}
}
}
}

void Control::dondeGirar(double d)
{
double a = getAnguloActual();
double equis =0;
if (d > a){
    equis = d - a;

    if(equis<180)
giroDer(d);
    else
giroIzq(d);

  }
else{
    equis = a-d;

    if (equis<180)
     giroIzq(d);
    else
     giroDer(d);

  }
}
float Control::temperatureCelcius(int mlx) 
{
  int dev = mlx;
  int data_low = 0;
  int data_high = 0;
  int pec = 0;
  
  i2c_start_wait(dev+I2C_WRITE);
  i2c_write(0x07);
  
  i2c_rep_start(dev+I2C_READ);
  data_low = i2c_readAck();
  data_high = i2c_readAck();
  pec = i2c_readNak();
  i2c_stop();
  
  double tempFactor = 0.02;
  double tempData = 0x0000;
  int frac;
  
  tempData = (double)(((data_high & 0x007F) << 8) + data_low);
  tempData = (tempData * tempFactor)-0.01;
  float celcius = tempData - 273.15;
  
  return celcius;
}
void Control::avanzar(double deseado, double dIzq, double dDer, int &boostD)
{

double anguloA=0;
anguloA=getAnguloActual();

  int pwmIzquierda;
  int boost;
  int pwmDerecha;
  double error;
  double val1,val2;
  unsigned long ahora=millis();
  unsigned long CambioTiempo= ahora-pasado;
  lcd.clear();

  if(CambioTiempo >= TiempoMuestreo)
  {
    //calculo error
    if(deseado==0 && anguloA>=180)
    {
      error=360-anguloA;
    }
    else
    {
     error=deseado-anguloA;
    }

        //calculo integral
      errorPass=errorPass*2/3+error*TiempoMuestreo;
    // cálculo derivativo
       errorD =(error-errorAnt)/TiempoMuestreo;
    ////Serial.print("Error: ");
    ////Serial.println(error);

      float P;
      float I;
      float D;

      P=kp*error;                                // control proporcional
      I=ki*errorPass;                            // control Integral
      D=kd*errorD;
      boost=P+I+D;



      pasado=ahora;                // actualizar tiempo
      errorAnt=error;              // actualizar el error

  }

lcd.setCursor(0,0);

/*
  if(dIzq<19 || dDer<19)
{
bool izq=false;
double dif=0;
if(dIzq<dDer){
val1=tofIE();
val2=tofIA();
izq=true;
}
else{
val1=tofDA();
val2=tofDE();
}
/*if(val1==-1)
{
if(izq)
lcd.print("izqE");
else
lcd.print("derA");
}
if(val2==-1)
{
if(izq)
lcd.print("izqA");
else
lcd.print("derE");
}
*/
/*
if(val1!=-1 && val2!=-1)
dif=val1-val2;
else
dif=0;
if(val1>300 || val2>300)
dif=0;
if(abs(dif)>300)
dif=0;
boost+=dif;
}
*/
if(dIzq==357)
dIzq=2;
if(dDer==357)
dDer=2;

if(dIzq==iPasado)
conta++;
else
conta=0;

if(conta>=3)
{
  dIzq=30;
  dDer=30;
}

if(dIzq>=2 && dIzq<=5){
boost+=(55-dIzq*3);
if(!bT)
boostD=1;
}
else if(dIzq>=10 && dIzq<15)
{
boost-=dIzq*5.7;
if(!bT)
{
  if(dIzq>14)
boostD=3;
else 
boostD=2;

}
}
else if(dDer>=2 && dDer<=5)
{boost-=(55-dDer*3);
if(!bT)
boostD=1;
}
else if (dDer>=10 && dDer<15)
{
boost+=dDer*5.7;
if(!bT)
{
if(dDer>14)
boostD=3;
else 
boostD=2;
}
}
else
{
boost+=0;
boostD=0;
}
    pwmIzquierda=base+boost;
    pwmDerecha=base-boost;

 if(pwmDerecha>255)
pwmDerecha=255;
if(pwmIzquierda>255)
pwmIzquierda=255;

if(pwmDerecha<limit)
  pwmDerecha=limit;
if(pwmIzquierda<limit)
pwmIzquierda=limit;

            digitalWrite(motorIzqAde2, LOW);
            analogWrite(motorIzqAde1, pwmIzquierda);
            digitalWrite(motorIzqAtras1, LOW);
            analogWrite(motorIzqAtras2, pwmIzquierda);
            digitalWrite(motorDerAde1, LOW);
            analogWrite(motorDerAde2, pwmDerecha);
            digitalWrite(motorDerAtras1, LOW);
            analogWrite(motorDerAtras2, pwmDerecha);
            
iPasado = dIzq;
//Serial.print("actual: ");
//Serial.println(anguloA);
//Serial.print("deseado: ");
//Serial.println(deseado);
//Serial.print("error: ");
//Serial.println(error);
}

void Control::atrasPID(double deseado)
{
double anguloA=0;
anguloA=getAnguloActual();

  int pwmIzquierda;
  int boost;
  int pwmDerecha;
  double error;
  unsigned long ahora=millis();
  unsigned long CambioTiempo= ahora-pasado;

  if(CambioTiempo >= TiempoMuestreo)
  {
    //calculo error
    if(deseado==0 && anguloA>=180)
    {
      error=360-anguloA;
    }
    else
    {
     error=deseado-anguloA;
    }

        //calculo integral
      errorPass=errorPass*2/3+error*TiempoMuestreo;
    // cálculo derivativo
       errorD =(error-errorAnt)/TiempoMuestreo;
    ////Serial.print("Error: ");
    ////Serial.println(error);

      float P;
      float I;
      float D;

      P=kp*error;                                // control proporcional
      I=ki*errorPass;                            // control Integral
      D=kd*errorD;
      boost=P+I+D;



      pasado=ahora;                // actualizar tiempo
      errorAnt=error;              // actualizar el error

  }


    pwmIzquierda=base+boost;
    pwmDerecha=base-boost;

 if(pwmDerecha>255)
pwmDerecha=255;
if(pwmIzquierda>255)
pwmIzquierda=255;

if(pwmDerecha<60)
  pwmDerecha=60;
if(pwmIzquierda<60)
pwmIzquierda=60;

digitalWrite(motorIzqAde1, LOW);
analogWrite(motorIzqAde2, pwmDerecha);
digitalWrite(motorIzqAtras2, LOW);
analogWrite(motorIzqAtras1, pwmDerecha);
digitalWrite(motorDerAde2, LOW);
analogWrite(motorDerAde1, pwmIzquierda);
digitalWrite(motorDerAtras2, LOW);
analogWrite(motorDerAtras1, pwmIzquierda);

}
void Control::atras()
{
digitalWrite(motorIzqAde1, LOW);
analogWrite(motorIzqAde2, base-7);
digitalWrite(motorIzqAtras2, LOW);
analogWrite(motorIzqAtras1, base-7);
digitalWrite(motorDerAde2, LOW);
analogWrite(motorDerAde1, base+7);
digitalWrite(motorDerAtras2, LOW);
analogWrite(motorDerAtras1, base+7);
}

void Control::atrasSN()
{
  girosX=0;
  this -> setBase(velInicial);
  this -> atrasPID(de);
  delay(600);
  this -> detenerse();
  delay(50);
  this -> actualizaSetPoint(de);
  delay(100);
  this -> setBase(velInicial);
  
  rightCount=0;
  
  while(rightCount< tic1/5)
  this -> avanzar(de,30,30,bD);
  
  this -> detenerse();
}

void Control::setup(){

bno.begin();
bno.setExtCrystalUse(true);

lcd.init();
lcd.backlight();
i2c_init(); 

escribirLCDabajo("LCD lista");
lcd.clear();
lcd.setCursor(0,0);

pinMode(pin1, INPUT); 
pinMode(pin2, INPUT);
pinMode(pin3, INPUT);
pinMode(pin4, INPUT);
pinMode(pin5, INPUT);
pinMode(pin6, INPUT);
pinMode(37,OUTPUT);
pinMode(39,OUTPUT);
/*
while(orientationStatus() != 3)
  {
    lcd.setCursor(6,0);
    lcd.print("NCal");
  }
    lcd.setCursor(6,1);
    lcd.print("SCal");


  delay(2700);
 */

  pinMode(motorIzqAde1, OUTPUT);
  pinMode(motorIzqAde2, OUTPUT);
  pinMode(motorIzqAtras1, OUTPUT);
  pinMode(motorIzqAtras2, OUTPUT);
  pinMode(motorDerAde1, OUTPUT);
  pinMode(motorDerAde2, OUTPUT);
  pinMode(motorDerAtras1, OUTPUT);
  pinMode(motorDerAtras2, OUTPUT);

  sensors_event_t event;
  bno.getEvent(&event);
  setPointY=event.orientation.z;
  
  //Serial.print("set point z: ");
  //Serial.println(setPointY);

 Wire.begin();

 sensor.init();
 sensor.setTimeout(500);



#if defined LONG_RANGE
  sensor.setSignalRateLimit(0.1);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  sensor.setMeasurementTimingBudget(15500);
#elif defined HIGH_ACCURACY
  sensor.setMeasurementTimingBudget(155000);
#endif

}

void Control::tcaselect(int i)
{
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();

delay(20);

}

void Control::actualizaSetPointY()
{
  double valu = y();
  setPointY+=valu;
}
void Control::atras1()
{
  this -> setBase(180);
  this -> atras();
  delay(600);
  this -> detenerse();
  delay(50);
  this -> actualizaSetPoint(de);
  delay(100);
  this -> setBase(velInicial);
  
  rightCount=0;
  while(rightCount<tic/5)
  this -> avanzar(de,30,30,bD);
  
  this -> detenerse();
}

void Control::choqueIzq(double d, int var)
{
double nuevoD;
if(d==0)
nuevoD=360-var;
else
nuevoD=d-var;

giro(nuevoD,1);
atrasPID(nuevoD);
delay(300);
giro(d,1);
detenerse();
//delay(100);

checa(d);
}
void Control::choqueDer(double d, int var)
{
bool be=false;
double nuevoD=d+var;

giro(nuevoD,1);
atrasPID(nuevoD);
delay(300);
giro(d,1);
detenerse();
//delay(100);

checa(d);
}
void Control::adelante(int i, int d)
{
            digitalWrite(motorIzqAde2, LOW);
            analogWrite(motorIzqAde1, i);
            digitalWrite(motorIzqAtras1, LOW);
            analogWrite(motorIzqAtras2, i);
            digitalWrite(motorDerAde1, LOW);
            analogWrite(motorDerAde2, d);
            digitalWrite(motorDerAtras1, LOW);
            analogWrite(motorDerAtras2, d);
}
void Control::moveAdelanteFast(){
            digitalWrite(motorIzqAde2, LOW);
            analogWrite(motorIzqAde1, 255);
            digitalWrite(motorIzqAtras1, LOW);
            analogWrite(motorIzqAtras2, 255);
            digitalWrite(motorDerAde1, LOW);
            analogWrite(motorDerAde2, 255);
            digitalWrite(motorDerAtras1, LOW);
            analogWrite(motorDerAtras2, 255);
}
void Control::desacomodo(int n)
{
            digitalWrite(motorIzqAde2, LOW);
            analogWrite(motorIzqAde1, 255);
            digitalWrite(motorIzqAtras1, LOW);
            analogWrite(motorIzqAtras2, 255);
            digitalWrite(motorDerAde1, LOW);
            analogWrite(motorDerAde2, LOW);
            digitalWrite(motorDerAtras1, LOW);
            analogWrite(motorDerAtras2, LOW);

            delay(n);

            digitalWrite(motorIzqAde2, LOW);
            analogWrite(motorIzqAde1, LOW);
            digitalWrite(motorIzqAtras1, LOW);
            analogWrite(motorIzqAtras2, LOW);
            digitalWrite(motorDerAde1, LOW);
            analogWrite(motorDerAde2, 255);
            digitalWrite(motorDerAtras1, LOW);
            analogWrite(motorDerAtras2, 255);

            delay(n);
}
void Control::moveAdelante()
{
            digitalWrite(motorIzqAde2, LOW);
            analogWrite(motorIzqAde1, base-7);
            digitalWrite(motorIzqAtras1, LOW);
            analogWrite(motorIzqAtras2, base-7);
            digitalWrite(motorDerAde1, LOW);
            analogWrite(motorDerAde2, base+7);
            digitalWrite(motorDerAtras1, LOW);
            analogWrite(motorDerAtras2, base+7);

}
void Control::detenerse(){
  uint8_t i=235;
  
digitalWrite(motorIzqAde1, HIGH);
digitalWrite(motorIzqAde2, HIGH);
digitalWrite(motorIzqAtras1, HIGH);
digitalWrite(motorIzqAtras2, HIGH);
digitalWrite(motorDerAde1, HIGH);
digitalWrite(motorDerAde2, HIGH);
digitalWrite(motorDerAtras1, HIGH);
digitalWrite(motorDerAtras2, HIGH);

delay(10);

digitalWrite(motorIzqAde1, LOW);
digitalWrite(motorIzqAde2, LOW);
digitalWrite(motorIzqAtras1, LOW);
digitalWrite(motorIzqAtras2, LOW);
digitalWrite(motorDerAde1, LOW);
digitalWrite(motorDerAde2, LOW);
digitalWrite(motorDerAtras1, LOW);
digitalWrite(motorDerAtras2, LOW);
}

void Control::moveAtras(){
digitalWrite(motorIzqAde1, LOW);
analogWrite(motorIzqAde2, 255);
digitalWrite(motorIzqAtras2, LOW);
analogWrite(motorIzqAtras1, 255);
digitalWrite(motorDerAde2, LOW);
analogWrite(motorDerAde1, 255);
digitalWrite(motorDerAtras2, LOW);
analogWrite(motorDerAtras1, 255);
}

bool Control::cuadroNegro()
{
return false;
tcaselect(1);
  int r, g, b;

  tcs.getRawData(&r, &g, &b);

Serial.println(r);
Serial.println(g);
Serial.println(b);
Serial.println(" ");

if(r<360 && g<360 && b<360)
return true;
else
return false;

}

void Control:: escribirNumLCD(int num)
{
  lcd.clear();
  lcd.print(num);
}

void Control:: escribirLetraLCD(char letra)
{
 // lcd.clear();
  lcd.print(letra);
}

void Control::printLoc(int x, int y, int z)
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("x:");
  lcd.setCursor(2,0);
  lcd.print(x);

   lcd.setCursor(7,0);
  lcd.print("y:");
  lcd.setCursor(9,0);
  lcd.print(y);

  lcd.setCursor(11,0);
  lcd.print("z:");
  lcd.setCursor(13,0);
  lcd.print(z);

delay(100);
}

void Control::checar()
{
  this -> checa(de);
}
void Control::blinkLeds()
{
  for(int i = 0; i < 5; i++)
  {
    digitalWrite(37, HIGH);
    delay(500);
    digitalWrite(37, LOW);
    digitalWrite(39, HIGH);
    delay(500);
    digitalWrite(39, LOW);
  }
}
