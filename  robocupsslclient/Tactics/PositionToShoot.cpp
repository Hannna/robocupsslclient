/*
 * PositionToShoot.cpp
 *
 *  Created on: Feb 3, 2012
 *      Author: maciek
 */

#include "PositionToShoot.h"
#include "../EvaluationModule/EvaluationModule.h"
#include "../Task/Task.h"
#include "../Task/GoToPose.h"
#include "../Task/KickBall.h"
#include "../Task/GoToBall.h"
#include "../Task/MoveBall.h"

PositionToShoot::PositionToShoot(Robot & robot): Tactic(robot)
{
	this->active = false;
}


void PositionToShoot::execute(void *){

	LOG_INFO(log,"Start PositionToShoot tactic " );
    EvaluationModule& evaluation=EvaluationModule::getInstance();

    Task::predicate predicates = Task::null;
    Task::status taskStatus = Task::not_completed;
    //Pose goalPose;
    Pose goalPose  ;//= evaluation.findBestDribbleTarget(robot.getRobotName(), robot.getRobotID());
    while( !this->stop ){
    	taskStatus = Task::not_completed;

    	//if( goalPose.distance(Config::getInstance().field.FIELD_MIDDLE_POSE)  < 1.0)
    	//	break;
    	LOG_INFO(log,"end calculate best dribble target goTo "<<goalPose );

		this->currentTask = TaskSharedPtr( new 	GoToPose( goalPose.getPosition(), &robot ) );
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
		}

		if( taskStatus == Task::ok ){
			break;
		}

    }
    finished = true;
    LOG_INFO(log,"exit from DribbleToShoot tactic " );
}

bool  PositionToShoot::isFinish(){

    return this->finished;
}

PositionToShoot::~PositionToShoot()
{

}

