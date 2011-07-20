/*
 * GoToBall.cpp
 *
 *  Created on: 25-04-2011
 *      Author: maciek
 */

#include "GoToBall.h"
#include "GetBall.h"

GoToBall::GoToBall(Robot * robot_):Task(robot_) {

	this->video.updateGameState( this->currGameState );
	this->ballPose = this->currGameState->getBallPos();
	this->goToPose =  new GoToPose( this->ballPose, robot);

}

Task* GoToBall::nextTask(){

	//jesli pilka jest na oucie to czekaj
	if( evaluationModule.getBallState(this->robot->getRobotID()) == EvaluationModule::out){
		LOG_INFO(log,"out. change GoToBall -> NULL");
		return NULL;
	}
	//jesli pilka jest zajeta przez naszych
	if( evaluationModule.getBallState(this->robot->getRobotID()) == EvaluationModule::occupied_our){
		LOG_INFO(log,"occupied_our change GoToBall -> NULL");
		return NULL;
	}
	//jesli pilka jest zajeta przez przeciwnika
	if( evaluationModule.getBallState(this->robot->getRobotID()) == EvaluationModule::occupied_theirs){
		LOG_INFO(log,"occupied_theirs change GoToBall -> NULL");
		return NULL;
	}

	//jesli pilka jest wolna jedz do niej
	if( evaluationModule.getBallState(this->robot->getRobotID()) == EvaluationModule::free){
		Pose nextBallPose = ballPose + ( this->currGameState->getBallGlobalVelocity()*this->video.getUpdateDeltaTime() );
		double dist = ballPose.distance( this->currGameState->getRobotPos( robot->getRobotID()) );
		//jesli pilka jest bliska to chwyc ja
		if( dist  <= GetBall::maxDistanceToBall ){
			LOG_INFO(log,"change GoToBall -> GetBall");
			return new GetBall(robot);
		}
		//LOG_INFO(log,"change GoToBall -> NULL");
		return NULL;
	}

	return NULL;
}


Task::status GoToBall::run(void * arg, int steps){

	this->video.updateGameState( this->currGameState );

	//jesli pilka jest na oucie to czekaj
	if( (this->predicates & Task::analyse_all_field) == 0){
		if( evaluationModule.getBallState(this->robot->getRobotID()) == EvaluationModule::out){
			LOG_INFO(log,"change GoToBall -> NULL");
			return Task::error;
		}
	}

	if( ballPose.distance(this->currGameState->getBallPos()) > GetBall::maxDistanceToBall){
		ballPose = this->currGameState->getBallPos();
		delete this->goToPose;
		Vector2D ballPos = this->currGameState->getBallPos().getPosition();
		ballPos = ballPos + ( this->currGameState->getBallGlobalVelocity() * Videoserver::getInstance().getUpdateDeltaTime( ) );

		if( (this->predicates & Task::analyse_all_field ) > 0){
			this->goToPose =  new GoToPose( Pose(ballPos,0), robot, true);
		}
		else
			this->goToPose =  new GoToPose( Pose(ballPos,0), robot );
			//this->goToPose =  new GoToPose( this->currGameState->getBallPos(), robot );
	}

	return this->goToPose->execute(arg,steps);
}



GoToBall::~GoToBall() {
	delete goToPose;
}
