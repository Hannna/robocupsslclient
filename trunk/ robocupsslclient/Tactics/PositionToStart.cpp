/*
 * PositionToStart.cpp
 *
 *  Created on: 09-05-2011
 *      Author: maciek
 */

#include "PositionToStart.h"
#include "../Task/GoToPose.h"
#include "../Exceptions/SimulationException.h"

PositionToStart::PositionToStart( const Pose &startPose_, Robot& robot_  ): Tactic(robot_), startPose( startPose_ ) {
	// TODO Auto-generated constructor stub

}

PositionToStart::PositionToStart( const Pose &startPose_, Robot* robot_  ): Tactic(*robot_), startPose( startPose_ ) {
	// TODO Auto-generated constructor stub

}


bool PositionToStart::isFinish(){
	return this->finished;
}

void PositionToStart::execute(void* null){

	GameStatePtr gameState ( new GameState() );
	Task::status taskStatus = Task::not_completed;
	Task* newTask;

	while( !stop && taskStatus!=Task::ok ){
		if( Videoserver::getInstance().updateGameState( gameState ) <0 ){
    		std::ostringstream s;
    		s<<__FILE__<<":"<<__LINE__;
			throw SimulationException(s.str());
		}

		bool force = true;
		this->currentTask = TaskSharedPtr( new GoToPose( startPose, &robot, force  ) );

		while(!stop && taskStatus!=Task::ok){
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
				finished = true;
				return;
			}
		}
	}
	robot.stop();

	finished = true;

}

PositionToStart::~PositionToStart() {
	this->stop = true;

	std::cout<<" exit from positionToStart "<<std::endl;
	this->join();
	// TODO Auto-generated destructor stub
}
