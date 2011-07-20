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

    while(true){
    	//jesli pilke maja nasi
    	if( evaluation.getBallState( robot.getRobotID() ) == EvaluationModule::occupied_our ){
    		//jesli pilke mam ja
    		if( evaluation.isRobotOwnedBall(robot) ){
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
					return;
				}
			}
			//Nie jest potrzebne bo wskaznik jest przechowywany w smartPtr currentTask
			//if(newTask)
			//	delete newTask;
		}
    }
}

bool Receive_pass::isFinish(){

	return true;
}

Receive_pass::~Receive_pass() {
	// TODO Auto-generated destructor stub
}
