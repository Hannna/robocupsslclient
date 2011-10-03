/*
 * NaivePlay.cpp
 *
 *  Created on: 08-05-2011
 *      Author: maciek
 */

#include "NaivePlay.h"

#include "../Tactics/Tactic.h"
#include "../Tactics/ShootTactic.h"
#include "../Tactics/Pass.h"
#include "../Tactics/DefendLine.h"
#include "../Tactics/PositionToStart.h"

#include "../Role/Role.h"

/*
PLAY Naive Offense
APPLICABLE offense
DONE aborted !offense
ROLE 1
 shoot A
 none
ROLE 2
 defend_point {-1400 250} 0 700
 none
ROLE 3
 defend_lane {B 0 -200} {B 1175 -200}
 none
ROLE 4
 defend_point {-1400 -250} 0 1400
 none
*/


NaivePlay::NaivePlay(std::string teamColor): Play(teamColor,3) {

	this->roles[0]->addTactic( new ShootTactic( roles[0]->getRobot() ) );

	this->roles[1]->addTactic( new ShootTactic( roles[1]->getRobot() ) );

	this->roles[2]->addTactic( new ShootTactic( roles[2]->getRobot() ) );

}

bool NaivePlay::isFinished(){

	return true;
}

void NaivePlay::execute(){

	LOG_INFO(log,"STARTING NaivePlay");

	if( this->roles[0]->getTacticsSize() == 0 )
		this->roles[0]->addTactic( new ShootTactic( roles[0]->getRobot() ) );

	this->roles[0]->execute();

	if( this->roles[1]->getTacticsSize() == 0 )
		this->roles[1]->addTactic( new ShootTactic( roles[1]->getRobot() ) );

	this->roles[1]->execute();

	if( this->roles[2]->getTacticsSize() == 0 )
		this->roles[2]->addTactic( new ShootTactic( roles[2]->getRobot() ) );

	this->roles[2]->execute();


}

void NaivePlay::reset(){

}


void NaivePlay::waitForFinish( ){
//	std::list<AbstractTactic *>::iterator tactic = tactics.begin();

//	for( ; tactic!= tactics.end();tactic++  ){
//		(*tactic)->start(NULL);
//	}

//	tactic = tactics.begin();
//	for( ; tactic!= tactics.end();tactic++  ){
//		(*tactic)->join();
//	}

//	tactic = tactics.begin();
//	for( ; tactic!= tactics.end();tactic++  ){
//		delete *tactic;
//	}

//	tactics.clear();
	while( this->roles[0]->getCurrentTactic()->isFinish() ){
		//this->role0.currentTactic->isFinish();
		usleep(100);

	}

	while( this->roles[1]->getCurrentTactic()->isFinish() ){
			//this->role0.currentTactic->isFinish();
		usleep(100);

	}

	while( this->roles[2]->getCurrentTactic()->isFinish() ){
			//this->role0.currentTactic->isFinish();
		usleep(100);

	}
	//std::cout<<"Preparing for start finished"<<std::endl;

}



/*wszystkie roboty zatrzymuja sie co najmniej 30 cm od pilki
 *
 */
void NaivePlay::stop(){

	LOG_INFO(log,"STOP PLAY for "<<this->teamColor);

	this->roles[0]->stop();
	this->roles[1]->stop();
	this->roles[2]->stop();

}

void NaivePlay::halt(){

	LOG_INFO(log,"halt NaivePlay");

	this->roles[0]->disperse( );
	this->roles[1]->disperse( );
	this->roles[2]->disperse( );
	//role3->getRobot()->stop();
}


NaivePlay::~NaivePlay() {

}
