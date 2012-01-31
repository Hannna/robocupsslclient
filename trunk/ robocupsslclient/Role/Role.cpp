/*
 * Role.cpp
 *
 *  Created on: 24-05-2011
 *      Author: maciek
 */

#include "Role.h"
#include <boost/foreach.hpp>
#include <boost/bind.hpp>

#include "../Tactics/Tactic.h"
#include "../Tactics/DisperseTactic.h"
#include "../Logger/Logger.h"

Role::Role( Robot* robot_ ): log( getLoggerPtr( robot_!=NULL ? robot_->getRobotName().c_str() : "app_debug" ) ) {
	this->robot = robot_;
	currentTactic = NULL;
	this->finished = false;
}

void Role::stop( ) {
	LOG_INFO(log,"try stop role for robot "<<this->robot->getRobotName());
	if( this->currentTactic )
		this->currentTactic->stopTactic();

	this->currentTactic->waitForFinish();

	if(this->robot){
		//roboty odjezdzaja od siebie podczas kolizji
		/*
		//Vector2D Robot::repulsivePotentialField(Vector2D positionCoordinates, std::list< Vector2D > obstacles)
		std::list< Vector2D >obstacles;
		Pose currRobotPose;
		while( true ){
			GameStatePtr gameState;

			Videoserver::getInstance().updateGameState( gameState );
			const std::vector<std::string> blueTeam=Config::getInstance().getBlueTeam();

			BOOST_FOREACH(std::string modelName,blueTeam){
				if(modelName.compare( robot->getRobotName() )!=0){
					obstacles.push_back( gameState->getRobotPos( Robot::getRobotID( modelName) ).getPosition() );
				}
				else
					currRobotPose = gameState->getRobotPos( Robot::getRobotID( modelName) );
			}

			const std::vector<std::string> redTeam=Config::getInstance().getRedTeam();
			BOOST_FOREACH(std::string modelName,redTeam){
				if(modelName.compare( robot->getRobotName() )!=0){
					obstacles.push_back( gameState->getRobotPos( Robot::getRobotID(modelName) ).getPosition() );
				}
				else
					currRobotPose = gameState->getRobotPos( Robot::getRobotID( modelName) );
			}

			Vector2D velocity = this->robot->repulsivePotentialField(currRobotPose.getPosition(),obstacles);
			this->robot->setGlobalSpeed(velocity,0,currRobotPose.get<2>() );
		}
		*/
	}
	this->robot->stop();
	LOG_INFO(log,"stop role for robot "<<this->robot->getRobotName());
}

void Role::addTactic( Tactic * tactic){
	tactics.push_front(tactic);
}

void Role::executeNextTactic(){
	LockGuard lock(mutex);
	//std::cout<<"executeNextTactic for "<<this->robot->getRobotName()<<std::endl;
	LOG_INFO(log,"executeNextTactic for "<<this->robot->getRobotName());

	if( !tactics.empty() ){
		if( currentTacticIter!=tactics.end() ){
			(*this->currentTacticIter)->stopTactic();
			//LOG_INFO(log,"tactic is going to stop for "<<this->robot->getRobotName());

			(*this->currentTacticIter)->waitForFinish();
			LOG_INFO(log,"tactic is finished for "<<this->robot->getRobotName());
			(*this->currentTacticIter)->reset( );

			currentTacticIter++;
			//LOG_INFO(log,"go to next tactic for "<<this->robot->getRobotName());
		}
		else{
			currentTacticIter=tactics.end();
			currentTactic = NULL;
			finished =true;
			return;
		}

	}

	if( currentTacticIter!=tactics.end() ){
		currentTactic = *currentTacticIter;
		if( currentTactic ){
			//currentTactic->reset();
			currentTactic->start( );
		}
	}
	else{
		currentTacticIter=tactics.end();
		currentTactic = NULL;
		finished =true;
	}

}

size_t Role::getTacticsSize(){
	if(this->currentTactic)
	 return this->tactics.size() + 1;
	else return this->tactics.size();
}

void Role::execute(){
	LockGuard lock(mutex);
	if( !tactics.empty() ){
		currentTactic = tactics.front();
		currentTacticIter = tactics.begin();
		//tactics.pop_front();
	}

	if( currentTactic )
		currentTactic->start( );

	//delete currentTactic;

	//currentTactic  = NULL;

	LOG_INFO(log,"finished  role execution for robot "<<this->robot->getRobotName() );
}

void Role::disperse( ){

	currentTactic = new DisperseTactic( *this->robot );
	currentTactic->start( );

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
