/*
 * DribbleToShoot.cpp
 *
 *  Created on: Jan 29, 2012
 *      Author: maciek
 */
#include "../EvaluationModule/EvaluationModule.h"
#include "../Task/Task.h"
#include "../Task/GoToPose.h"
#include "../Task/KickBall.h"
#include "../Task/GoToBall.h"
#include "../Task/MoveBall.h"

#include "DribbleToShoot.h"


DribbleToShoot::DribbleToShoot(Robot & robot): Tactic(robot)
{
	this->active = true;
}


void DribbleToShoot::execute(void *){

	LOG_INFO(log,"Start DribbleToShoot tactic " );
    EvaluationModule& evaluation=EvaluationModule::getInstance();

    Task::predicate predicates = Task::null;
    Task::status taskStatus = Task::not_completed;
    Pose goalPose;

    while( !this->stop ){
    	taskStatus = Task::not_completed;
    	Pose goalPose  = evaluation.findBestDribbleTarget(robot.getRobotName(), robot.getRobotID());
    	LOG_INFO(log,"end calculate best dribble target " );

		this->currentTask = TaskSharedPtr( new MoveBall( goalPose, &robot ) );
		Task::predicate p = Task::kick_if_we_can;
		this->currentTask->markParam(p);
		this->currentTask->markParam(predicates);
		//bestScore = score;
		Task* newTask;
		int steps=1;
		while( taskStatus!=Task::ok && !this->stop ){
			newTask = this->currentTask->nextTask();

			if( newTask ){
				this->currentTask = TaskSharedPtr( newTask );
			}

			taskStatus = this->currentTask->execute(NULL,steps);

			if( taskStatus == Task::error ){
				LOG_ERROR(log,"Shoot tactic Task::error " <<taskStatus );
				robot.stop();
				break;
			}
			else
			if( taskStatus == Task::collision ){
				robot.stop();
				LOG_FATAL(log,"Shoot tactic Task::collision " );
				finished = true;
				return;
			}
			else
			if( taskStatus == Task::kick_ok ){
				robot.stop();
				LOG_FATAL(log," Shoot tactic Task::kick_ok " );
				finished = true;
				return;
			}
			if( taskStatus == Task::movingBallForbidden ){
				robot.stop();
			}

			if( taskStatus == Task::get_ball ){
				taskStatus = Task::ok;
				predicates = Task::got_ball;
			}

		}

    }
    finished = true;
    LOG_INFO(log,"exit from DribbleToShoot tactic " );
}

bool  DribbleToShoot::isFinish(){

    return this->finished;
}

DribbleToShoot::~DribbleToShoot()
{

}
