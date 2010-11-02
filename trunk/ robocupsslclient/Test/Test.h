/*
 * test.h
 *
 *  Created on: 2010-01-03
 *      Author: maciek
 */

#ifndef TEST_H_
#define TEST_H_

#include "../VideoServer/Videoserver.h"
//#include "src/Logger/Logger.h"
#include "../Robot/Robot.h"


extern const std::string ifaceName;

typedef struct threadArg{
	Pose goalPose;
	Videoserver * video;
	Robot* robot;
} * threadArgPtr;
//testuje predkosc w srodowisku gazebo
void testVel(Vector2D speed,double yaw,Robot& robot,time_t testTime);

//testuje odczytywanie predkosci w srodowisku gazebo
void testPose(Robot& robot,Pose newPose);

//testuje odczytywanie rotacji
void testRotation(Videoserver & video,Robot& robot);

//testuje obliczanie przyspieszenia
void checkAcceleration(Vector2D speed, Videoserver & video,Robot& robot);

//testuje algorytm rrt
void testRRT(Pose goalPose,Videoserver & video,Robot* robot);

//testuje algortym rrt jako odpalany w osobnym watku
void testRRTThread(Videoserver & video);

void* RRTThread(void * arg);

void testMotion(Pose goalPose,Videoserver & video,Robot& robot);

// testowanie rozwiazania wielowatkowego
 // dla kazdego robota tworzony jest osobny watek
 // dodatkowo videoserwer uruchamiany jest jako osobny watek
 //
 //
void testTaskThread();

void* testTask(void * arg);

#endif /* TEST_H_ */
