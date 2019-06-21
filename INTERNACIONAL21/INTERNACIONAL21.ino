#include "Movement.h"

Movement all;
SoftwareSerial camara777(51, 50);

void dispenserEncoder()
{
  all.maze.robot.dispenser.rightCount++;
}

void rightEncoderEvent()
{
  all.maze.sensors.motors.rightCount++;
}

void infiniteLoop()
{
  int a = 0;
  bool p = true;
  all.maze.moveForward(p);
  all.maze.sensors.motors.escribirLCD("END OF ROUND", "ROBORREGOS 2019");

  while(a == 0)
    delay(8000);
}

void visualVictim()
{
  all.maze.camaraChar='0';

if(camara777.available()>0)
all.maze.camaraChar = camara777.read();

}


void setup() {
  Serial.begin(9600);
  camara777.begin(115200);
  pinMode(37,OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(all.maze.robot.dispenser.RH_ENCODER_A), dispenserEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2),rightEncoderEvent, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(50),visualVictim, CHANGE);
  all.maze.setup();
  all.maze.robot.dispenser.setup();
  all.maze.sensors.setup();
  all.maze.sensors.motors.setup();
  all.maze.sensors.motors.atras1();

  if(all.maze.sensors.getDistanceOf(all.maze.sensors.echo_A, 4) > 20)
  {
    all.maze.robot.tile[all.maze.robot.x][all.maze.robot.y][all.maze.robot.z].down(0, &all.maze.robot.tile[all.maze.robot.x+1][all.maze.robot.y][all.maze.robot.z]);
    all.maze.robot.tile[all.maze.robot.x+1][all.maze.robot.y][all.maze.robot.z].exist(1);
    all.maze.robot.tile[all.maze.robot.x+1][all.maze.robot.y][all.maze.robot.z].pending(1);
  }
  else
    all.maze.robot.tile[all.maze.robot.x][all.maze.robot.y][all.maze.robot.z].down(1, &all.maze.robot.tile[all.maze.robot.x+1][all.maze.robot.y][all.maze.robot.z]);
  
  all.maze.robot.changeSquare(&all.maze.robot.tile[all.maze.robot.x][all.maze.robot.y][all.maze.robot.z], all.maze.sensors.getDistanceOf(all.maze.sensors.echo_D, 2), all.maze.sensors.getDistanceOf(all.maze.sensors.echo_I, 1), all.maze.sensors.getDistanceOf(all.maze.sensors.echo_E, 3), all.maze.robot.getDirection());
 all.maze.robot.changeStatus(&all.maze.robot.tile[all.maze.robot.x][all.maze.robot.y][all.maze.robot.z], all.maze.robot.getDirection());


//  Serial.println(all.maze.sensors.getDistanceOf(sensors.echo_A, 4));
 
}
void moveC()
{
  visualVictim();
  all.moveOn();
}
void loop() {

  if(all.maze.robot.z==0)
  digitalWrite(37,HIGH);
  else
  digitalWrite(37,LOW);

  
  (all.endOfRound) ? infiniteLoop() : all.moveOn();
}
