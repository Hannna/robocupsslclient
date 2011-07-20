/*
 * Role.cpp
 *
 *  Created on: 24-05-2011
 *      Author: maciek
 */

#include "Role.h"
#include "../Tactics/Tactic.h"
#include "../Logger/Logger.h"

Role::Role( Robot* robot_ ): log( getLoggerPtr( robot_!=NULL ? robot_->getRobotName().c_str() : "app_debug" ) ) {
	this->robot = robot_;
	currentTactic = NULL;
}

void Role::stop( ) {
	this->currentTactic->stopTactic();
	if(this->robot)
		this->robot->stop();

	LOG_INFO(log,"stop role for robot "<<this->robot->getRobotName());
}

void Role::addTactic( Tactic * tactic){
	tactics.push_front(tactic);
}

void Role::execute(){
	if( !tactics.empty() ){
		currentTactic = tactics.front();
		tactics.pop_front();
	}

	if( currentTactic )
		currentTactic->start(NULL);

}

Role::~Role() {
	if(this->currentTactic)
		delete currentTactic;
	if( tactics.empty() ){
		std::list<Tactic* >::iterator ii = this->tactics.begin();
		for(;ii!=this->tactics.end();ii++){
			delete *ii;
		}
	}

}
