/*
 * GetBall.cpp
 *
 *  Created on: 24-04-2011
 *      Author: maciek
 */

#include "GetBall.h"
#include "GoToBall.h"

GetBall::GetBall(Robot * robot):Task(robot) {
	// TODO Auto-generated constructor stub

}

Task* GetBall::nextTask(){

	GameStatePtr currGameState(new GameState());
	video.updateGameState(currGameState);
	Pose robotPos = currGameState->getRobotPos(this->robot->getRobotID() );
	Pose ballPos=currGameState->getBallPos();
	Vector2D toBall = Vector2D( ballPos.getPosition() - robotPos.getPosition() );

	if( toBall.length() > this->maxDistanceToBall ){
		LOG_INFO(log, "change GetBall ->> GoToBall " );
		return new GoToBall(this->robot);
	}
	return NULL;
}

Task::status GetBall::run(void * arg, int steps){

	GameStatePtr currGameState(new GameState());
	Pose currRobotPose;
	Pose ballPos;

	Vector2D reference;
	Vector2D toBall;
	double angle;
	Vector2D robotCurrentVel;
	while( !this->stopTask && (steps--)!=0 ){

		video.updateGameState(currGameState);
		robotCurrentVel = currGameState->getRobotVelocity(this->robot->getRobotID());

		//bool ballIsOwned =  false;
		currRobotPose = currGameState->getRobotPos(this->robot->getRobotID() );
		ballPos=currGameState->getBallPos();
		toBall = Vector2D( ballPos.getPosition() - currRobotPose.getPosition() );
		if( toBall.length() > this->maxDistanceToBall ){
			LOG_INFO(log, "GetBall, ball is to far " );
			return Task::not_completed;
		}

		reference = Vector2D( cos( currRobotPose.get<2>()+M_PI_2 ), sin( currRobotPose.get<2>()+M_PI_2 ) );

		angle = toBall.angleTo(reference);

		//czy pilka jest przed dribblerem
		if ( fabs(angle) < 0.33 ){
			//czy pilka jest odpowienio blisko dribblera
			if ( toBall.length() < ( 0.075+0.012+0.022 ) / cos(angle) ){
				LOG_INFO(log, "GetBall Task::ok" );
				return Task::ok;
			}//podjedz do pilki
			else{
				Vector2D robotNewVel=calculateVelocity( robotCurrentVel, currRobotPose, ballPos);
				robot->setRelativeSpeed( robotNewVel, 0 );
				LOG_INFO(log, "GetBall podjedz do pilki " );
				return Task::not_completed;
			}
		}//obroc robota
		else{
			double w = robot->calculateAngularVel( *currGameState,robot->getRobotID(), ballPos);
			robot->setRelativeSpeed( Vector2D(0,0), w );
			LOG_INFO(log, "obrot robota do pilki " );
			return Task::not_completed;
		}
	}

	return Task::ok;
}

GetBall::~GetBall() {
	// TODO Auto-generated destructor stub
}
