/*
 * SimplePlay.cpp
 *
 *  Created on: 08-06-2011
 *      Author: maciek
 */

#include "SimplePlay.h"

#include "../Tactics/Tactic.h"
#include "../Tactics/ShootTactic.h"
#include "../Tactics/Pass.h"
#include "../Tactics/DefendLine.h"
#include "../Tactics/PositionToStart.h"

#include "../Role/Role.h"

/*
PLAY SimplePlay
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


SimplePlay::SimplePlay(std::string teamColor): Play(teamColor,3) {

	this->roles[0]->addTactic( new ShootTactic( roles[0]->getRobot() ) );

	this->roles[1]->addTactic( new ShootTactic( roles[1]->getRobot() ) );

	this->roles[2]->addTactic( new ShootTactic( roles[2]->getRobot() ) );

}

bool SimplePlay::isFinished(){

	return true;
}

void SimplePlay::execute(){

	LOG_INFO(log,"STARTING SIMPLEPLAY");

	this->roles[0]->execute();

	this->roles[1]->execute();

	this->roles[2]->execute();


}

void SimplePlay::reset(){

}


void SimplePlay::waitForFinish( ){
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

	std::cout<<"Preparing for start finished"<<std::endl;

}



/*wszystkie roboty zatrzymuja sie co najmniej 30 cm od pilki
 *
 */
void SimplePlay::stop(){

	LOG_INFO(log,"STOP PLAY for "<<this->teamColor);

	this->roles[0]->stop();
	this->roles[1]->stop();
	this->roles[2]->stop();

}


SimplePlay::~SimplePlay() {

}
