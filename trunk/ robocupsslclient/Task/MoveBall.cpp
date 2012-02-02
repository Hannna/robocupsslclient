/*
 * MoveBall.cpp
 *
 *  Created on: 24-04-2011
 *      Author: maciek
 */

#include "MoveBall.h"
#include "GoToBall.h"
#include "KickBall.h"

MoveBall::MoveBall(const Pose & pose,Robot * robot_):Task(robot_),goalPose(pose),maxRobotDistanceWithBall(Config::getInstance().getMaxRobotDistanceWithBall()) {

    GameStatePtr currGameState( new GameState() );
    video.updateGameState(currGameState);
	startPose = currGameState->getRobotPos(this->robot->getRobotID());
	previousRobotPose = startPose;
	goToPose = new GoToPose(goalPose.getPosition(),robot_);
	goToPose->markParam(Task::kick_if_we_can);
	distanceWithBall=0;
}

Task* MoveBall::nextTask(){

	//sprawdz czy robot ma pilke
	if( !this->evaluationModule.isRobotOwnedBall( this->robot ) ){
		LOG_INFO(this->log,"MoveBall -> GoToBall");
		//jesli robot nie ma pilki to zmieni task na GoToBall
		return new GoToBall( this->robot );
	}
	else if( distanceWithBall >= maxRobotDistanceWithBall ){
		LOG_INFO(this->log," maxRobotDistanceWithBall exceeded. Change MoveBall -> KickBall");
		//jesli robot nprzekroczyl maksymalny dystans poruszania sie z pilka kopnij ja do celu i podjedz
		Task* task = new KickBall( robot, goalPose.getPosition(), goalPose.get<2>() ,Config::getInstance().getDribbleKickForce());
		task->markParam(Task::kick_for_dribble);
		return task;
	}
	//jesli robot ma pilke to jedz z pilka do celu
	else{
		//jesli ustawiono flage zezwalajaca na strzal
		if( (this->predicates & Task::kick_if_we_can) > 0){
			//oblicz czy wato strzelic na bramke
		    double angleToGoal = 0;
		    double score = 0;
		    std::pair<double, double> ang=this->evaluationModule.aimAtGoal(robot->getRobotName(),angleToGoal,score);

			LOG_INFO(log, "current position score = "<<score<<" ang.first "<<ang.first<<" ang.second "<<ang.second );

			//jesli warto strzelic na bramke
			if( fabs(score) > EvaluationModule::minOpenAngle   ){
				LOG_INFO(this->log,"MoveBall -> KickBall score "<<score<<"  ang.first"<<ang.first<<" ang.second "<<ang.second );
				//return new KickBall( robot,  convertAnglePI( ang.first + sign*score/2.0 )  ) ;
				Vector2D targetPosition;
				if( this->robot->isBlue() )
					targetPosition = Videoserver::getBlueGoalMidPosition();

				if( this->robot->isRed() )
					targetPosition = Videoserver::getRedGoalMidPosition();

				return new KickBall( robot, targetPosition, angleToGoal, Config::getInstance().getMaxKickForce());


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

    GameStatePtr currGameState( new GameState() );
    double currSimTime=video.updateGameState(currGameState);
	Pose currentRobotPose=currGameState->getRobotPos(this->robot->getRobotID());

	distanceWithBall+=currentRobotPose.distance(this->previousRobotPose);
	previousRobotPose = currentRobotPose;
	if( distanceWithBall >= maxRobotDistanceWithBall ){
		LOG_INFO(this->log,"MoveBall, moving with ball forbidden");
		//robot->stop();
		return Task::movingBallForbidden;
	}
	return goToPose->execute(NULL,steps);
	//return Task::ok;
}

MoveBall::~MoveBall() {
	if(goToPose)
		delete goToPose;
}
