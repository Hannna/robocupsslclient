/*
 * Recevive_pass.cpp
 *
 *  Created on: 24-04-2011
 *      Author: maciek
 */
#include "Receive_pass.h"
#include "../EvaluationModule/EvaluationModule.h"
#include "../Task/GoToPose.h"
#include "../Task/Rotate.h"
#include "../Exceptions/SimulationException.h"


Receive_pass::Receive_pass(Robot& robot_):Tactic(robot_) {
	LOG_INFO(log,"Tactic Receive_pass for robot  " <<robot_.getRobotName() );

}

void Receive_pass::execute(void*){

    EvaluationModule& evaluation=EvaluationModule::getInstance();
    GameStatePtr gameState ( new GameState() );
    Pose goalPose;

	LOG_INFO(log,"START Tactic Receive_pass for robot  " <<robot.getRobotName() );
    while( true ){
	   {
		   LockGuard l(this->mutex);
		   if(this->stop)
			   break;
	   }
    	//jesli pilke maja nasi
    	EvaluationModule::ballState bs = evaluation.getBallState( robot.getRobotID() );

    	if(  bs == EvaluationModule::occupied_our || bs == EvaluationModule::mine ){
    		//jesli pilke mam ja
    		//if( evaluation.isRobotOwnedBall(robot) ){
    		if( bs == EvaluationModule::mine ){
    			robot.stop();
    			break;
    		}

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
				LockGuard l(this->mutex);
				if(this->stop)
					break;

				newTask = this->currentTask->nextTask();

				if(newTask){
					this->currentTask = TaskSharedPtr( newTask );
				}
				int steps=-1;
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
}

bool Receive_pass::isFinish(){
	LockGuard m(mutex);
	return this->finished;
}

Receive_pass::~Receive_pass() {
	// TODO Auto-generated destructor stub
}
