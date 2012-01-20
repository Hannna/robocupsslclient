/*
 * Experiment1.cpp
 *
 *  Created on: Oct 2, 2011
 *      Author: maciek
 */

#include "Experiment1.h"
#include "Play.h"
#include "../VideoServer/Videoserver.h"
#include "../Tactics/FollowLineAndAvoidObs.h"

Experiment_1::Experiment_1( std::string teamColor, const int nrOfROles, const std::string iter_): Play( teamColor, nrOfROles ),iter(iter_) {
	// TODO Auto-generated constructor stub

}

void Experiment_1::execute( ){

	LOG_INFO(log,"starting experiment_1 play  " <<this->teamColor);
	//getcurrent robot position
	GameStatePtr gameState( new GameState() );
	Videoserver::getInstance().updateGameState( gameState );

	Vector2D startPostion( 2.7 , 1.3);//( 2.7 , 1.0)// gameState->getRobotPos( this->roles[0]->getRobot().getRobotID( ) ).getPosition();
	Vector2D endPostion( 2.7 , 6.1);//( 2.7 , 6.7)
	//rola 1 jazda wzdluz linii w gore
	RoleIterator ii = this->roles.begin();
	for( ; ii != this->roles.end();ii++){
		ii->second->addTactic( new
			FollowLineAndAvoidObs( ii->second->getRobot(), startPostion, endPostion, this->iter ) );
		ii->second->execute();
	}

}

void Experiment_1::stop(){
	Play::RoleIterator ii = this->roles.begin();
	for(;ii != this->roles.end();ii++ ){
		if( ii->second->getCurrentTactic() )
			ii->second->getCurrentTactic()->stopTactic();
	}
}

void Experiment_1::waitForFinish( ){
	//	tactics.clear();

	Play::RoleIterator ii = this->roles.begin();
	for(;ii != this->roles.end();ii++ ){
		if( ii->second->getCurrentTactic() )
			while( !ii->second->getCurrentTactic()->isFinish() ){
				usleep(100);

			}
	}
	LOG_INFO(log,"all Roles finished  " <<this->teamColor);
}

void Experiment_1::reset(){

}

Experiment_1::~Experiment_1() {
	// TODO Auto-generated destructor stub
}


