/*
 * KickBall.cpp
 *
 *  Created on: 2010-06-07
 *      Author: maciek
 */

#include "KickBall.h"

KickBall::KickBall(Robot * robot): Task(robot) {


}

bool KickBall::run(void * arg, int steps ){
	this->robot->kick();
	return true;
}

KickBall::~KickBall() {

}
