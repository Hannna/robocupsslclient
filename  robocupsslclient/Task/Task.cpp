/*
 * Task.cpp
 *
 *  Created on: 2010-06-07
 *      Author: maciek
 */

#include "Task.h"

Task::Task(Robot* robot_):video(Videoserver::getInstance()), robot(robot_) {
	// TODO Auto-generated constructor stub
	stopTask=false;
}

void Task::stop(){
	this->stopTask=true;
}

Task::~Task() {
	// TODO Auto-generated destructor stub
}
