/*
 * FollowLine.cpp
 *
 *  Created on: Oct 2, 2011
 *      Author: maciek
 */

#include "FollowLine.h"
#include "../EvaluationModule/EvaluationModule.h"
#include "../Task/Task.h"
#include "../Task/GoToPose.h"
#include "../Task/KickBall.h"
#include "../Task/GoToBall.h"
#include "../Task/MoveBall.h"
#include "../Exceptions/SimulationException.h"


FollowLine::FollowLine(Robot & robot_, const Vector2D p1_, const Vector2D p2_ ): Tactic(robot_),
				p1( p1_ ), p2( p2_ ) {
	this->finished = false;
	LOG_INFO(log,"create FollowLine tactic for robot "<<robot_.getRobotName()<< " Line "<<p1<<" "<<p2);
}

void FollowLine::execute(void *){

	LOG_INFO(log,"create start FollowLine tactic. Line "<<p1<<" "<<p2);

	Task::status taskStatus = Task::not_completed;

    Pose goalPose = Pose(this->p1,0);
    Pose nextGoalPose = Pose(this->p2,0);

    //znajdz punkt na odcinku (p1,p2) bedacy najblizej pilki
    //i nie nalezacy do zadnej przeszkody

    //r-nie prostej przechodzacej przez P1 i P2
  //  double a = -( p2.y - p1.y )/( p2.x - p1.x );
 //   double b = 1;
 //   double c = (-1)*p1.x * a - p1.y;

    GameStatePtr gameState ( new GameState() );
    if( Videoserver::getInstance().updateGameState( gameState ) < 0 )
    	throw SimulationException("FollowLine::execute");


    while(true){
	   taskStatus = Task::not_completed;

	   if( Videoserver::getInstance().updateGameState( gameState ) < 0)
		   throw SimulationException("FollowLine::execute");

	   Pose currPose = gameState->getRobotPos(this->robot.getRobotID());
	   Vector2D diff = currPose.getPosition() - goalPose.getPosition();

	   //podazaj do kranca zdefiniowanego odcinka
	   if( ( fabs(diff.x) < 0.05 ) && ( fabs(diff.y) < 0.05 ) ){
		   Pose tmp = goalPose;
		   goalPose = nextGoalPose;
		   nextGoalPose = tmp;
	   }

		Vector2D robotCurrentGlobalVel=gameState->getRobotGlobalVelocity( robot.getRobotID() );

		//robotNewVel=calculateVelocity( robotCurrentVel, Pose(targetRelPosition.x,targetRelPosition.y,0));

		Vector2D robotNewGlobalVel=this->robot.calculateVelocity( robotCurrentGlobalVel, currPose, goalPose);
		//double w = robot->calculateAngularVel(*currGameState,robot->getRobotID(), goalPose);
		bool haveBall = false;
		double w = robot.calculateAngularVel( gameState->getRobotPos( robot.getRobotID() ), goalPose, gameState->getSimTime(), haveBall );

	   //this->robot.setRelativeSpeed( Vector2D( 1.0,0.0 ) * sgn( diff.x ) ,0 );
	   this->robot.setGlobalSpeed( robotNewGlobalVel ,w , currPose.get<2>());

	   /*
	   while( ( fabs(diff.x) > 0.05 ) && ( fabs(diff.y) > 0.05 ) ){
		  // newTask = this->currentTask->nextTask();

		   if( diff.x < 0 )
			   this->robot.setRelativeSpeed( Vector(1.0,0.0),0 );
		   if(){
				this->currentTask = TaskSharedPtr( newTask );
			}
			break;
	   }
	   */
	}
    this->finished = false;
}

bool FollowLine::isFinish(){

	return this->finished;
}

FollowLine::~FollowLine() {
	// TODO Auto-generated destructor stub
}

