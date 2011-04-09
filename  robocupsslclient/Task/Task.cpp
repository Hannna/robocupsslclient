/*
 * Task.cpp
 *
 *  Created on: 2010-06-07
 *      Author: maciek
 */

#include "Task.h"

Task::Task(Robot* robot_):video(Videoserver::getInstance()), robot(robot_), log(getLoggerPtr (robot_->getRobotName().c_str() ) ) {
	stopTask=false;
}

void Task::stop(){
	mutex_.lock();
	this->stopTask=true;
	robot->setRelativeSpeed(Vector2D(0.0,0.0),0);
	mutex_.unlock();
}


bool Task::execute(void* arg, int steps){
	bool stop=false;
	bool res=false;
	do{
		mutex_.lock();
		if(!this->stopTask);
			stop=true;
		mutex_.unlock();
		res=this->run(arg, steps);
	}
	while( !stop || !(res) );

	return res;
}


Task::~Task() {
	// TODO Auto-generated destructor stub
}
