/*
 * DIsperseTactic.cpp
 *
 *  Created on: Aug 9, 2011
 *      Author: maciek
 */

#include "DisperseTactic.h"

#include "../EvaluationModule/EvaluationModule.h"
#include "../Task/Task.h"
#include "../Task/Disperse.h"

DisperseTactic::DisperseTactic(Robot & robot): Tactic(robot)
{

}


void DisperseTactic::execute(void *){

	LOG_INFO(log," start disperseTactic " );

    Task::status taskStatus = Task::not_completed;

    while( !this->stop && taskStatus != Task::ok){
    	taskStatus = Task::not_completed;
    	LOG_INFO(log," before  Disperse Task" );
		this->currentTask = TaskSharedPtr( new Disperse( &robot ) );
		LOG_INFO(log," after  Disperse Task" );
		int steps=1;
		taskStatus = this->currentTask->execute(NULL,steps);
		LOG_INFO(log," after  execute disperse Disperse " );
		if( taskStatus == Task::ok){
			finished = true;
		}

		LOG_INFO(log," disperseTactic " );
    }


    LOG_INFO(log,"exit from disperse tactic " );
}

bool  DisperseTactic::isFinish(){

    return this->finished;
}


DisperseTactic::~DisperseTactic() {
	// TODO Auto-generated destructor stub
}
