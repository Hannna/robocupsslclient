/*
 * GoToPosition.cpp
 *
 *  Created on: Feb 4, 2012
 *      Author: maciek
 */

#include "GoToPosition.h"
#include "../Task/GoToPose.h"

GoToPosition::GoToPosition(Robot & robot_, const Pose & goalPosition_): Tactic(robot_), goalPosition( goalPosition_ ) {
	// TODO Auto-generated constructor stub
	this->active = false;

}
void GoToPosition::execute(void *){

	LOG_INFO(log,"create start GoToPosition tactic. Go to "<<goalPosition);

	Task::status taskStatus = Task::not_completed;

	this->currentTask = TaskSharedPtr( new GoToPose( goalPosition.getPosition(), &robot ) );
    while(taskStatus!=Task::ok){
       {
    	LockGuard l(this->mutex);
    	if(this->stop)
    		break;
       }
	   taskStatus = Task::not_completed;
	   int steps =1;
	   while( taskStatus!=Task::ok && !this->stop ){
		   taskStatus = this->currentTask->execute(NULL,steps);
	   }


	}
    LOG_FATAL( log,"############# TACTIC COMPLETED #############" );
    this->robot.stop();
    LockGuard l(this->mutex);
    this->finished = true;
    pthread_cond_broadcast(&this->finish_cv);
}

bool GoToPosition::isFinish(){
	LockGuard l(this->mutex);
	return this->finished;
}
GoToPosition::~GoToPosition() {
	// TODO Auto-generated destructor stub
}

