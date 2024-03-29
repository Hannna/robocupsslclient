/*
 * Task.cpp
 *
 *  Created on: 2010-06-07
 *      Author: maciek
 */

#include "Task.h"

std::ostream & operator<<(std::ostream & os, const Task::status & status ){

	switch(status){
		case Task::collision:
			os<<"Task::collision";
			break;
		case Task::error:
			os<<"Task::error";
			break;
		case Task::not_completed:
			os<<"Task::not_completed";
			break;
		case Task::ok:
			os<<"Task::ok";
			break;
		case Task::kick_ok:
			os<<"Task::kick_ok";
			break;
		default:
			os<<"Task:: unknown";
			break;
	}
	return os;
}

Task::Task(Robot* robot_):video(Videoserver::getInstance()), robot(robot_),
							log(getLoggerPtr (robot_->getRobotName().c_str() ) ),
							evaluationModule( EvaluationModule::getInstance() ){
	stopTask=false;
	this->currGameState=GameStatePtr( new GameState() );
	this->predicates = 0;
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

void Task::markParam(predicate p){
	this->predicates |=p;

}

void Task::unmarkParam(predicate p){

	this->predicates &= ~p;

}

Task::~Task() {
	// TODO Auto-generated destructor stub
}
