/*
 * Pass.cpp
 *
 *  Created on: 24-04-2011
 *      Author: maciek
 */

#include "Pass.h"
#include "../Task/KickBall.h"
#include "../Task/Rotate.h"
#include "../Task/GoToBall.h"
#include "../Exceptions/SimulationException.h"

Pass::Pass( Robot& robot_, const Robot::robotID targetRobotID_ ): Tactic(robot_),targetRobotID(targetRobotID_) {
	LOG_INFO(log,"Tactic pass for robot  " <<robot_.getRobotName()<<" to robot "<<targetRobotID_ );
	this->active = true;

}

void Pass::execute(void*){

    EvaluationModule& evaluation=EvaluationModule::getInstance();
    GameStatePtr gameState ( new GameState() );
    Pose goalPose;

	LOG_INFO(log,"START Tactic pass for robot  " <<robot.getRobotName()<<" to robot "<<targetRobotID);
    while(true){
 	   {
 		   LockGuard l(this->mutex);
 		   if(this->stop)
 			   break;
 	   }

    	if( Videoserver::getInstance().updateGameState( gameState ) < 0 ){
    		std::ostringstream s;
    		s<<__FILE__<<":"<<__LINE__;
    		throw SimulationException(s.str());
    	}

		Task::status taskStatus = Task::not_completed;
		Task* newTask;

		double rotationToTarget = 0;
		if( evaluation.isRobotOwnedBall(robot) ){
			if( evaluation.aimAtTeamMate( robot.getRobotID(), targetRobotID, &rotationToTarget ) > 0.5 ){
				Pose target = Pose( gameState->getRobotPos( targetRobotID ).getPosition(),rotationToTarget) ;
				this->currentTask = TaskSharedPtr( new KickBall( &robot, target ) );
			}
			else{
				LOG_INFO(log," aim at team mate return "<< evaluation.aimAtTeamMate( robot.getRobotID(), targetRobotID ) );
				goalPose = gameState->getRobotPos( targetRobotID );
				this->currentTask = TaskSharedPtr( new Rotate( goalPose.getPosition(), &robot  ) );
				this->currentTask->markParam( Task::should_have_ball );
			}
		}
		else{
			this->currentTask = TaskSharedPtr( new GoToBall( &robot  ) );
			this->currentTask->markParam(Task::analyse_all_field);
		}

		while(taskStatus!=Task::ok){
			newTask = this->currentTask->nextTask();

			if(newTask){
				this->currentTask = TaskSharedPtr( newTask );
			}
			int steps=1;
			taskStatus = this->currentTask->execute(NULL,steps);

			if( taskStatus == Task::error ){
				LOG_ERROR(log,"Tactic error taskStatus " <<taskStatus );

				break;
			}

			if( taskStatus == Task::collision ){
				robot.stop();
				LOG_FATAL(log,"Tactic error taskStatus " <<taskStatus );

				LOG_FATAL( log,"############# TACTIC COMPLETED #############" );
				robot.stop();
				//LockGuard m(mutex);
				//this->finished = true;
				//pthread_cond_broadcast(&this->finish_cv);
				//return;
				break;
			}

			if( taskStatus == Task::kick_ok ){
				robot.stop();
				LOG_FATAL(log,"Tactic taskStatus " <<taskStatus );

				robot.stop();
				LockGuard m(mutex);
				this->finished = true;
				pthread_cond_broadcast(&this->finish_cv);
				LOG_FATAL( log,"############# TACTIC COMPLETED #############" );
				return;
			}

			if( taskStatus == Task::get_ball ){
				robot.stop();
				break;
			}
		}
	}

	LockGuard m(mutex);
	this->finished = true;
	pthread_cond_broadcast(&this->finish_cv);
}

bool Pass::isFinish(){
	LockGuard m(mutex);
	//this->finished;
	return this->finished;
}

Pass::~Pass() {
	// TODO Auto-generated destructor stub
}
