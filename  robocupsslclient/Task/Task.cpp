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

Task::status Task::execute(void* arg,const  int steps_){
	int steps = steps_;
	bool stop=false;
	Task::status res = Task::ok;
	//do{
		mutex_.lock();
		if(!this->stopTask);
			stop=true;
		mutex_.unlock();
		res=this->run(arg, steps);

	//}
	//while( !stop  && steps-- !=0 );

	return res;
}


Task::~Task() {
	// TODO Auto-generated destructor stub
}
