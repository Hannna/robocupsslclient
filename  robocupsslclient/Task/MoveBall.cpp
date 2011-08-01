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
			double score;
			double sign = 1.0;

			if( ang.first * ang.second > 0.0  ){
				score = ang.second - ang.first;
			}
			else{
				if( fabs(ang.second )  + fabs( ang.first) > M_PI ){
					score = fabs( M_PI - fabs(ang.second )  + M_PI- fabs( ang.first) );
					sign = -1;
				}
				else{
					score = fabs(ang.second )  + fabs( ang.first);
				}
			}
			//( (ang.first * ang.second) > 0.0 ) ?  ang.second - ang.first   : fabs( fabs(ang.second )  + M_PI- fabs( ang.first)   );

			//LOG_INFO(log, "current position score = "<<score<<" ang.first "<<ang.first<<" ang.second "<<ang.second );

			//jesli warto strzelic na bramke
			if( fabs(score) > EvaluationModule::minOpenAngle   ){
				LOG_INFO(this->log,"MoveBall -> KickBall score "<<score<<"  ang.first"<<ang.first<<" ang.second "<<ang.second );
				return new KickBall( robot,  convertAnglePI( ang.first + sign*score/2.0 )  ) ;
			}
			else{
				LOG_INFO(this->log,"MoveBall, change to KickBall not permitted. Score "<<score);
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
