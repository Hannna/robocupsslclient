/*
 * DefendLine.cpp
 *
 *  Created on: 02-05-2011
 *      Author: maciek
 */

#include "DefendLine.h"
#include "../EvaluationModule/EvaluationModule.h"
#include "../Task/Task.h"
#include "../Task/GoToPose.h"
#include "../Task/KickBall.h"
#include "../Task/GoToBall.h"
#include "../Task/MoveBall.h"
#include "../Exceptions/SimulationException.h"

DefendLine::DefendLine(Robot & robot_, const Vector2D p1_, const Vector2D p2_, const double maxDistFromLine_ ): Tactic(robot_),
				p1( p1_ ), p2( p2_ ), maxDistFromLine( maxDistFromLine_ ) {

	LOG_INFO(log,"create DefendLine tactic. Line "<<p1<<" "<<p2);
}

void DefendLine::execute(void *){

	Task::status taskStatus = Task::not_completed;

    Pose goalPose = Pose(this->p1,0);

    //znajdz punkt na odcinku (p1,p2) bedacy najblizej pilki
    //i nie nalezacy do zadnej przeszkody

    //r-nie prostej przechodzacej przez P1 i P2
    double a = -( p2.y - p1.y )/( p2.x - p1.x );
    double b = 1;
    double c = (-1)*p1.x * a - p1.y;

    Vector2D ballPosition;
    GameStatePtr gameState ( new GameState() );
    if( Videoserver::getInstance().updateGameState( gameState ) < 0 )
    	throw SimulationException("DefendLine::execute");

    ballPosition = gameState->getBallPos().getPosition();

    goalPose = Pose( ballPosition.projectionOn(a,b,c), 0);


    while(true){
	   taskStatus = Task::not_completed;

	   if( Videoserver::getInstance().updateGameState( gameState ) < 0)
		   throw SimulationException("DefendLine::execute");

	   ballPosition = gameState->getBallPos().getPosition();
	   goalPose = Pose( ballPosition.projectionOn(a,b,c), 0);

	   this->currentTask = TaskSharedPtr( new GoToPose( goalPose, &robot ) );
	   //bestScore = score;
	   Task* newTask;

	   while(taskStatus!=Task::ok){
		   newTask = this->currentTask->nextTask();

		   if(newTask){
				this->currentTask = TaskSharedPtr( newTask );
			}
			int steps=1;
			taskStatus = this->currentTask->execute(NULL,steps);

			if( taskStatus == Task::error ){
				break;
			}

			if( taskStatus == Task::collision ){
				robot.stop();
				LOG_FATAL(log,"Defend Line Task::error ");
				return;
			}
			break;
	   }

	   if( goalPose.distance(Pose(this->p1,0)) < 0.01 )
		   goalPose = Pose(this->p2,0);
	   else
		   goalPose = Pose(this->p1,0);
	}
}

bool DefendLine::isFinish(){

	return true;
}

DefendLine::~DefendLine() {
	// TODO Auto-generated destructor stub
}
