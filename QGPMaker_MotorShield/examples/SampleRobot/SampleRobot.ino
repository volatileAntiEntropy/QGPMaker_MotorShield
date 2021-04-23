/*
 Name:		SampleRobot.ino
 Created:	2021/3/7 22:41:14
 Author:	johnnyQin
*/
//#define DEBUG

#include "RobotSystem.hpp"

QGPMaker::MotorShield shield;
QGPMaker::Robot robot;

// the setup function runs once when you press reset or power the board
void setup()
{
#ifdef DEBUG
	while (!Serial)
		;
	Serial.begin(9600);
#endif
	shield.begin(50);
	robot.setup();
}

// the loop function runs over and over again until power down or reset
void loop()
{
	robot.loop();
}
