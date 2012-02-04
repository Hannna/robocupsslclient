/*
 * Rotate.cpp
 *
 *  Created on: 07-05-2011
 *      Author: maciek
 */

#include "Rotate.h"
#include "GoToBall.h"


Rotate::Rotate( const Vector2D targetPosition_, Robot * robot ):Task(robot), targetPosition(targetPosition_),targetRobotId(Robot::unknown) {
	LOG_INFO(log,"create Rotate task, try to turn robot to position "<<targetPosition_ );
}

Rotate::Rotate( const Robot::robotID id, Robot * robot ):Task(robot),targetRobotId(id) {
	LOG_INFO(log,"create Rotate task, try to turn robot to "<<id<<" position " );
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
	if(this->targetRobotId != Robot::unknown)
		targetPosition = (*currGameState).getRobotPos(this->targetRobotId).getPosition();

	while( !this->stopTask && (steps--)!=0 ){

		if( lastSimTime < ( currSimTime=video.updateGameState(currGameState) ) ){
			lastSimTime = currSimTime;

			currRobotPose=currGameState->getRobotPos( robot->getRobotID() );
			bool haveBall = false;
			double angleToTarget = 0;
			double threshold = 0.017;

			if(this->predicates & Task::pass){
				bool canPass=this->evaluationModule.checkAngleToPass(targetPosition, currRobotPose, angleToTarget);
				threshold = 0.017 * currRobotPose.getPosition().distance(this->targetPosition);
				LOG_INFO(this->log,"set threshold to "<<threshold);

				if( canPass ){
					this->robot->stop();
					LOG_INFO(log,"Rotation to pass OK robot position "<<currRobotPose<<" target "<<targetPosition<<" angle to target "<<angleToTarget);
					return Task::ok;
				}
			}
			else{
				angleToTarget = calculateAngleToTarget( currRobotPose, Pose(targetPosition.x,targetPosition.y,0) );
				double threshold = 0.017; //1 stopien
				if( fabs( angleToTarget ) < threshold ){
					this->robot->stop();
					LOG_INFO(log,"Rotation OK robot position "<<currRobotPose<<" target "<<targetPosition<<" angle to target "<<angleToTarget);
					return Task::ok;
				}
			}
			double w = robot->calculateAngularVel( currRobotPose, targetPosition, currGameState->getSimTime(), haveBall );
			LOG_INFO(log,"move robot from"<<currRobotPose<<" to "<<targetPosition<<" setVel w "<<w<<" angle to target "<<angleToTarget);
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
