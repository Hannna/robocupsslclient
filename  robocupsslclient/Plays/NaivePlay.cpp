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


NaivePlay::NaivePlay(std::string teamColor): Play(teamColor) {

	this->role0.addTactic( new ShootTactic( role0.getRobot() ) );

	this->role1.addTactic( new ShootTactic( role1.getRobot() ) );

	this->role2.addTactic( new ShootTactic( role2.getRobot() ) );

}

bool NaivePlay::isFinished(){

	return true;
}

void NaivePlay::execute(){

	LOG_INFO(log,"STARTING SIMPLEPLAY");

	this->role0.execute();

	this->role1.execute();

	this->role2.execute();


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

	std::cout<<"Preparing for start finished"<<std::endl;

}



/*wszystkie roboty zatrzymuja sie co najmniej 30 cm od pilki
 *
 */
void NaivePlay::stop(){

	LOG_INFO(log,"STOP PLAY for "<<this->teamColor);

	this->role0.stop();
	this->role1.stop();
	this->role2.stop();

}


NaivePlay::~NaivePlay() {

}
