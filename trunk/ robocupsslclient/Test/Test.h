/*
 * test.h
 *
 *  Created on: 2010-01-03
 *      Author: maciek
 */

#ifndef TEST_H_
#define TEST_H_

#include "../VideoServer/Videoserver.h"
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

//testuje warstwe odpowiedzialna za sterowaniem robotem, wyhamowaniem przed celem itp
void testMotion(Pose goalPose,Videoserver & video,Robot& robot);

/*@brief testuje algortym rrt w srodowisku wieloagentowym
 *  wielu agentów ma za zadanie poruszac sie do dwolnych losowych punktow docelowych
 */
void testMultiRRTThread(Videoserver & video);

/*@brief testuje algortym rrt dla pojedynczego robota
 *  pozostale roboty sa statyczne, po osiagnieciu wyznaczonego celu
 *  lub kolizji funkcja konczy sie*
 */
void testSingleRRTThread(Videoserver & video);


// testowanie rozwiazania wielowatkowego
 // dla kazdego robota tworzony jest osobny watek
 // dodatkowo videoserwer uruchamiany jest jako osobny watek
 //
 //
void testTaskThread();

void* testTask(void * arg);

#endif /* TEST_H_ */
