#include "Movement.h"

Movement all;

void dispenserEncoder()
{
  all.maze.robot.dispenser.rightCount++;
}

void rightEncoderEvent()
{
  all.maze.sensors.motors.rightCount++;
}

void setup() {
  attachInterrupt(digitalPinToInterrupt(all.maze.robot.dispenser.RH_ENCODER_A), dispenserEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2),rightEncoderEvent, CHANGE);
  all.maze.robot.dispenser.setup();
  all.maze.sensors.setup();
  all.maze.sensors.motors.setup();
  all.maze.sensors.motors.atras1();
}

void loop() {
  all.moveOn();
}
