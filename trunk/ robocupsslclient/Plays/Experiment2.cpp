/*
 * Experiment2.cpp
 *
 *  Created on: Jan 28, 2012
 *      Author: maciek
 */

#include "Experiment2.h"

#include "Play.h"
#include "../VideoServer/Videoserver.h"

#include "../Tactics/Tactic.h"
#include "../Tactics/Receive_pass.h"
#include "../Tactics/PositionToPass.h"
#include "../Tactics/Pass.h"

Experiment_2::Experiment_2( std::string teamColor, const int nrOfROles): Play( teamColor, nrOfROles ) {
	// TODO Auto-generated constructor stub

}

void Experiment_2::updateState( ){

	Play::RoleIterator ii = this->roles.begin();
	bool changeTactics = false;

	for(;ii != this->roles.end();ii++ ){
		if( ii->second->getCurrentTactic()->isActiveTactic() ){
			if( ii->second->getCurrentTactic()->isFinish( ) ){
				changeTactics = true;
				std::cout<<"changeTactics"<<std::endl;
			}
		}
	}
	if(changeTactics){
		ii = this->roles.begin();
		for(;ii != this->roles.end();ii++ ){
			std::cout<<"executeNextTactic"<<std::endl;
			ii->second->executeNextTactic();
		}
	}
}

void Experiment_2::execute( ){

	LOG_INFO(log,"starting experiment_2 play  " <<this->teamColor);
	//getcurrent robot position
	GameStatePtr gameState( new GameState() );
	Videoserver::getInstance().updateGameState( gameState );

	//rola 1 jazda wzdluz linii w gore
	RoleIterator ii = this->roles.begin();

	ii->second->addTactic( new Receive_pass( ii->second->getRobot()) );
	ii->second->addTactic( new PositionToPass( ii->second->getRobot()) );
	ii->second->execute();

	Robot::robotID recvID = ii->second->getRobot().getRobotID();
	ii++;
	ii->second->addTactic( new Pass(ii->second->getRobot(), recvID ) );
	ii->second->execute();


	while(true){
		updateState( );
	}
	//receive_pass->stopTactic();
	//receive_pass->waitForFinish();


}

void Experiment_2::stop(){
	Play::RoleIterator ii = this->roles.begin();
	for(;ii != this->roles.end();ii++ ){
		if( ii->second->getCurrentTactic() )
			ii->second->getCurrentTactic()->stopTactic();
	}
}

void Experiment_2::waitForFinish( ){
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

void Experiment_2::reset(){

}

bool Experiment_2::isFinished( ){
	Play::RoleIterator ii = this->roles.begin();
	for(;ii != this->roles.end();ii++ ){
		if( ii->second->getCurrentTactic() )
			if( !ii->second->getCurrentTactic()->isFinish() ){
				return false;
			}
	}
	return true;
}

Experiment_2::~Experiment_2() {
	// TODO Auto-generated destructor stub
}


