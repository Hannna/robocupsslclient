/*
 * Experiments.cpp
 *
 *  Created on: Sep 27, 2011
 *      Author: maciek
 */

#include "Experiments.h"

#include "../Plays/Play.h"
#include "../Logger/Logger.h"

Experiment::Experiment(){
	;
};

void Experiment::prepare(){
	;
}

void Experiment::startExperiment(){
	this->start(NULL);
}

Experiment::~Experiment(){
	;
}

void Experiment::execute(void*){

	/*
	log4cxx::LoggerPtr log = getLoggerPtr ("app_debug");

	struct timespec req;
	req.tv_sec = 0;
	req.tv_nsec = 10000000;//10ms;
	struct timespec rem;
	bzero(&rem, sizeof(rem) );


	boost::shared_ptr<Play> redPlay;
	boost::shared_ptr<Play> bluePlay;

	while( true ){
		try{
			//ballState_ = EvaluationModule::getInstance().getBallState( Robot::red0 );
			//1. Get Command from sslbox
			//command = referee.getCommand();

			//teamID = referee.getTeamId();

			//LOG_INFO( log,"get command "<<command<<" for team "<<teamID );

			//rozpocznij gre
			redPlay = boost::shared_ptr<Play>( new NaivePlay("red") );
			bluePlay = boost::shared_ptr<Play>( new NaivePlay("blue") );
			//run in another thread
			bluePlay->execute();
			redPlay->execute();
		}
		catch( ... ){
			LOG_INFO( log, " in main STP loop exception handled " );
			break;
		}

		//SimControl::getInstance().restart();
		nanosleep(&req, &rem); //10ms
	}
	*/
}



