/*
 * Task.cpp
 *
 *  Created on: 2010-06-07
 *      Author: maciek
 */

#include "Task.h"

Task::Task(Robot* robot_):video(Videoserver::getInstance()), robot(robot_) {
	stopTask=false;
}

void Task::stop(){
	mutex_.lock();
	this->stopTask=true;
	mutex_.unlock();
}

bool Task::execute(){
	bool stop=false;
	bool res=false;
	do{
		mutex_.lock();
		if(!this->stopTask);
			stop=true;
		mutex_.unlock();
		res=this->run();
	}
	while( !stop || !(res) );

	return res;
}

Task::~Task() {
	// TODO Auto-generated destructor stub
}
