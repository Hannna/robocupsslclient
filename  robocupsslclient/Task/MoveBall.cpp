/*
 * MoveBall.cpp
 *
 *  Created on: 24-04-2011
 *      Author: maciek
 */

#include "MoveBall.h"
#include "GoToBall.h"
#include "KickBall.h"

MoveBall::MoveBall(const Pose & pose,Robot * robot_):Task(robot_),goalPose(pose) {

	goToPose = new GoToPose(goalPose,robot_);
	goToPose->markParam(Task::kick_if_we_can);
}

Task* MoveBall::nextTask(){

	//sprawdz czy robot ma pilke
	if( !this->evaluationModule.isRobotOwnedBall( this->robot ) ){
		LOG_INFO(this->log,"MoveBall -> GoToBall");
		//jesli robot nie ma pilki to zmieni task na GoToBall
		return new GoToBall( this->robot );
	}
	//jesli robot ma pilke to jedz z pilka do celu
	else{
		//jesli ustawiono flage zezwalajaca na strzal
		if( (this->predicates & Task::kick_if_we_can) > 0){
			//oblicz czy wato strzelic na bramke
			std::pair<double,double> ang=evaluationModule.aimAtGoal( robot->getRobotName() );

			//double score = fabs( ang.first - ang.second );
			double score =
			( (ang.first * ang.second) > 0 ) ? fabs( ang.first + ang.second ) : fabs( ang.first) + fabs(ang.second );

			//LOG_INFO(log, "current position score = "<<score<<" ang.first "<<ang.first<<" ang.second "<<ang.second );

			//jesli warto strzelic na bramke
			if( score > EvaluationModule::minOpenAngle   ){
				LOG_INFO(this->log,"MoveBall -> KickBall");
				return new KickBall( robot, ( ang.first + ang.second )/2  ) ;
			}
		}
		//LOG_INFO(this->log,"MoveBall -> GoToPose");
		//Task * task = new GoToPose( goalPose, this->robot );
		//task->markParam(Task::kick_if_we_can);
		//return task;
		return NULL;
	}
}
Task::status MoveBall::run(void * arg, int steps){
	return goToPose->execute(NULL,steps);
	//return Task::ok;
}

MoveBall::~MoveBall() {
	if(goToPose)
		delete goToPose;
}
