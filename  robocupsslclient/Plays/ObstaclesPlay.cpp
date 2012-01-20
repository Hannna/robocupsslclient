/*
 * Test1Play.cpp
 *
 *  Created on: Sep 29, 2011
 *      Author: maciek
 */

#include "ObstaclesPlay.h"
#include "Play.h"
#include "../VideoServer/Videoserver.h"
#include "../Tactics/FollowLine.h"

ObstaclesPlay::ObstaclesPlay(std::string teamColor, const int nrOfROles): Play(teamColor,nrOfROles) {
	// TODO Auto-generated constructor stub

}

void ObstaclesPlay::execute( ){

	LOG_INFO(log,"starting ObstaclesPlay  " <<this->teamColor);
	//getcurrent robot position
	GameStatePtr gameState( new GameState() );
	Videoserver::getInstance().updateGameState( gameState );

	//rola 0 stanie w punkcie
	//this->roles[0]->addTactic( new PositionToStart( robot0GoalPose, roles[0]->getRobot( ) ) );

	Vector2D startPostion = gameState->getRobotPos( this->roles[1]->getRobot().getRobotID( ) ).getPosition();
	//Vector2D startPostion
	//rola 1 jazda wzdluz linii w gore
	this->roles[1]->addTactic( new
			FollowLine( this->roles[1]->getRobot(), startPostion, startPostion + Vector2D(2.525,0) ) );
	this->roles[1]->execute();

	//rola 2 jazda wzdluz linii w dol
	startPostion = gameState->getRobotPos( this->roles[2]->getRobot().getRobotID( ) ).getPosition();
	this->roles[2]->addTactic( new
			FollowLine( this->roles[2]->getRobot(), startPostion, startPostion + Vector2D(2.525,0) ) );
	this->roles[2]->execute();

	//rola 3 jazda wzdluz linii w gore
	startPostion = gameState->getRobotPos( this->roles[3]->getRobot().getRobotID( ) ).getPosition();
	this->roles[3]->addTactic( new
			FollowLine( this->roles[3]->getRobot(), startPostion, startPostion + Vector2D(2.525,0) ) );
	this->roles[3]->execute();

	//rola 4 jazda wzdluz linii w dol
	startPostion = gameState->getRobotPos( this->roles[4]->getRobot().getRobotID( ) ).getPosition();
	this->roles[4]->addTactic( new
			FollowLine( this->roles[4]->getRobot(), startPostion, startPostion + Vector2D(2.525,0) ) );
	this->roles[4]->execute();

	//rola 5 jazda wzdluz linii w gore
	/*
	startPostion = gameState->getRobotPos( this->roles[5]->getRobot().getRobotID( ) ).getPosition();
	this->roles[5]->addTactic( new
			FollowLine( this->roles[5]->getRobot(), startPostion, startPostion + Vector2D(1.0,0) ) );
	this->roles[5]->execute();
	 */

	//rola 5 stanie w punckie
	//this->roles[5]->addTactic( new PositionToStart( robot0GoalPose, roles[0]->getRobot( ) ) );
}

void ObstaclesPlay::stop(){
	Play::RoleIterator ii = this->roles.begin();
	for(;ii != this->roles.end();ii++ ){
		if( ii->second->getCurrentTactic() )
			ii->second->getCurrentTactic()->stopTactic();
	}
}

void ObstaclesPlay::waitForFinish( ){
	//	tactics.clear();

	Play::RoleIterator ii = this->roles.begin();
	for(;ii != this->roles.end();ii++ ){
		if( ii->second->getCurrentTactic() )
			while( ii->second->getCurrentTactic()->isFinish() ){
				usleep(100);

			}
	}
}

void ObstaclesPlay::reset(){

}

ObstaclesPlay::~ObstaclesPlay() {
	// TODO Auto-generated destructor stub
}
