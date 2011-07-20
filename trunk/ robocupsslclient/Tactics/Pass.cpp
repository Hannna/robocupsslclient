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

}

void Pass::execute(void*){

    EvaluationModule& evaluation=EvaluationModule::getInstance();
    GameStatePtr gameState ( new GameState() );
    Pose goalPose;

    while(true){
    	if( Videoserver::getInstance().updateGameState( gameState ) < 0 ){
    		std::ostringstream s;
    		s<<__FILE__<<":"<<__LINE__;
    		throw SimulationException(s.str());
    	}

		Task::status taskStatus = Task::not_completed;
		Task* newTask;

		if( evaluation.isRobotOwnedBall(robot) ){
			if( evaluation.aimAtTeamMate( robot.getRobotID(), targetRobotID ) > 0.5 ){
				this->currentTask = TaskSharedPtr( new KickBall( &robot ) );
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
				return;
			}

			if( taskStatus == Task::kick_ok ){
				robot.stop();
				LOG_FATAL(log,"Tactic error taskStatus " <<taskStatus );
				return;
			}
		}
	}
}

bool Pass::isFinish(){
	return true;
}

Pass::~Pass() {
	// TODO Auto-generated destructor stub
}
