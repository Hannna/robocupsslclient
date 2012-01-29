/*
 * PositionToPass.cpp
 *
 *  Created on: Jan 28, 2012
 *      Author: maciek
 */

#include "PositionToPass.h"

#include "Receive_pass.h"
#include "../EvaluationModule/EvaluationModule.h"
#include "../Task/GoToPose.h"
#include "../Task/Rotate.h"
#include "../Exceptions/SimulationException.h"

PositionToPass::PositionToPass(Robot& robot_):Tactic(robot_) {
	LOG_INFO(log,"Tactic PositionToPass for robot  " <<robot_.getRobotName() );

}

void PositionToPass::execute(void*){

    EvaluationModule& evaluation=EvaluationModule::getInstance();
    GameStatePtr gameState ( new GameState() );
    Pose goalPose;

	LOG_INFO(log,"START Tactic PositionToPass for robot  " <<robot.getRobotName() );
    while( true ){
	   {
		   LockGuard l(this->mutex);
		   if(this->stop)
			   break;
	   }
    	//jesli pilke maja nasi
    	EvaluationModule::ballState bs = evaluation.getBallState( robot.getRobotID() );

		if( bs == EvaluationModule::mine ){
    			robot.stop();
    			break;
    	}

    	if(  ( bs == EvaluationModule::occupied_our) )
		{
    	    //Videoserver::getInstance().updateGameState( gameState );
    		if( Videoserver::getInstance().updateGameState( gameState ) < 0 ){
        		std::ostringstream s;
        		s<<__FILE__<<":"<<__LINE__;
    			throw SimulationException( s.str() );
    		}

    		goalPose = gameState->getBallPos();

    		Task::status taskStatus = Task::not_completed;

    		this->currentTask = TaskSharedPtr( new Rotate( goalPose.getPosition(), &robot  ) );

			Task* newTask;
			while(taskStatus!=Task::ok){
				{
					LockGuard l(this->mutex);
					if(this->stop)
						break;
				}

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
				else
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
			}
			//Nie jest potrzebne bo wskaznik jest przechowywany w smartPtr currentTask
			//if(newTask)
			//	delete newTask;
		}
    	//else
    	//	LOG_FATAL(log,"Nasz zawodnik nie ma pilki" );
    }

    LOG_FATAL( log,"############# TACTIC COMPLETED #############" );
	LockGuard m(mutex);
	this->finished = true;
	pthread_cond_broadcast(&this->finish_cv);
}

bool PositionToPass::isFinish(){
	LockGuard m(mutex);
	return this->finished;
}

PositionToPass::~PositionToPass() {
	// TODO Auto-generated destructor stub
}
