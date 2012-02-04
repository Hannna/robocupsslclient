/*
 * Recevive_pass.cpp
 *
 *  Created on: 24-04-2011
 *      Author: maciek
 */
#include "Receive_pass.h"
#include "../EvaluationModule/EvaluationModule.h"
#include "../Task/GoToPose.h"
#include "../Task/GoToBall.h"
#include "../Task/Rotate.h"
#include "../Exceptions/SimulationException.h"


Receive_pass::Receive_pass(Robot& robot_):Tactic(robot_) {
	LOG_INFO(log,"Tactic Receive_pass for robot  " <<robot_.getRobotName() );
	this->active = true;

}

void Receive_pass::execute(void*){

    EvaluationModule& evaluation=EvaluationModule::getInstance();
    GameStatePtr gameState ( new GameState() );
    Pose goalPose;

	LOG_INFO( log,"START Tactic receive pass for robot  " <<robot.getRobotName() );
    bool passRcv=false;
	while(!passRcv){
 	   {
 		   LockGuard l(this->mutex);
 		   if(this->stop)
 			   break;
 	   }

 		if( evaluation.getBallState(this->robot.getRobotID()) == BallState::mine ){
 			 passRcv=true;
 			 continue;
 		}

    	if( Videoserver::getInstance().updateGameState( gameState ) < 0 ){
    		std::ostringstream s;
    		s<<__FILE__<<":"<<__LINE__;
    		throw SimulationException(s.str());
    	}

		Task::status taskStatus = Task::not_completed;
		Task* newTask;

		this->currentTask = TaskSharedPtr( new GoToBall( &robot  ) );
		this->currentTask->markParam(Task::analyse_all_field);

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
				LockGuard m(mutex);
				this->finished = true;
				pthread_cond_broadcast(&this->finish_cv);
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
	robot.stop();
	pthread_cond_broadcast(&this->finish_cv);
}

bool Receive_pass::isFinish(){
	LockGuard m(mutex);
	return this->finished;
}

Receive_pass::~Receive_pass() {
	// TODO Auto-generated destructor stub
}
