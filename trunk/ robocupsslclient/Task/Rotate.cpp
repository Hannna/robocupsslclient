/*
 * Rotate.cpp
 *
 *  Created on: 07-05-2011
 *      Author: maciek
 */

#include "Rotate.h"
#include "GoToBall.h"


Rotate::Rotate( const Vector2D targetPosition_, Robot * robot ):Task(robot), targetPosition(targetPosition_) {
	LOG_INFO(log,"create Rotate task, try to turn robot to position "<<targetPosition_ );
}

Task* Rotate::nextTask(){

	if( this->predicates & Task::should_have_ball ){
		 if(!this->evaluationModule.isRobotOwnedBall( this->robot ) ){
			 LOG_INFO(log," change rotate task to goToBall " );
			 return new GoToBall( this->robot );
		 }

	}

	return NULL;
}

Task::status Rotate::run(void * arg, int steps){

	GameStatePtr currGameState( new GameState );
	double lastSimTime=0;
	double currSimTime=0;
	currSimTime=video.updateGameState(currGameState);
	//biezaca pozycja robota
	Pose currRobotPose = (*currGameState).getRobotPos( robot->getRobotID() );

	while( !this->stopTask && (steps--)!=0 ){

		if( lastSimTime < ( currSimTime=video.updateGameState(currGameState) ) ){
			lastSimTime = currSimTime;

			currRobotPose=currGameState->getRobotPos( robot->getRobotID() );
			bool haveBall = false;
			double angle = calculateAngleToTarget( currRobotPose, Pose(targetPosition.x,targetPosition.y,0) );
			double w = robot->calculateAngularVel( currRobotPose, targetPosition, currGameState->getSimTime(), haveBall );
			LOG_INFO(log,"move robot from"<<currRobotPose<<" to "<<targetPosition<<" setVel w "<<w<<" angle to target "<<angle);

			if( fabs(w) < 0.1 ){
				LOG_INFO(this->log,"Rotation OK return "<<Task::ok);
				return Task::ok;
			}
			robot->setRelativeSpeed( Vector2D(0,0), w );
		}
		usleep(100);
	}

	if(this->stopTask)
		return Task::ok;

	return Task::not_completed;
}


Rotate::~Rotate() {
	// TODO Auto-generated destructor stub
}
