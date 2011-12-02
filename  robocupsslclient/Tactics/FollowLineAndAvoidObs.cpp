/*
 * FollowLineAndAvoidObs.cpp
 *
 *  Created on: Oct 2, 2011
 *      Author: maciek
 */

#include "FollowLineAndAvoidObs.h"
#include "DefendLine.h"
#include "../EvaluationModule/EvaluationModule.h"
#include "../Task/Task.h"
#include "../Task/GoToPose.h"
#include "../Task/KickBall.h"
#include "../Task/GoToBall.h"
#include "../Task/MoveBall.h"
#include "../Task/GetBall.h"
#include "../Exceptions/SimulationException.h"

/*
std::pair<double, double> xConstraints( * )( ) {
	this->xConstraints = xConstraints;
}
*/
/*
void addYConstraint( std::pair<double, double> ( *yConstraints )( ) ){
	this->yConstraints = yConstraints;
}
*/

FollowLineAndAvoidObs::FollowLineAndAvoidObs(Robot & robot_, const Vector2D p1_, const Vector2D p2_ ): Tactic(robot_),
				p1( p1_ ), p2( p2_ ) {

	LOG_INFO(log,"create FollowLineAndAvoidObs tactic for robot "<<robot_.getRobotName()<< " Line "<<p1<<" "<<p2);
}

void FollowLineAndAvoidObs::execute(void *){
	LOG_INFO(log,"create start FollowLineAndAvoidObs tactic. Line "<<p1<<" "<<p2);

	EvaluationModule& evaluation=EvaluationModule::getInstance();

	Task::status taskStatus = Task::not_completed;
    GameStatePtr gameState ( new GameState() );
    if( Videoserver::getInstance().updateGameState( gameState ) < 0 )
    	throw SimulationException("FollowLineAndAvoidObs::execute");

    double maxXvalue ( Config::getInstance().field.FIELD_TOP_RIGHT_CORNER.x - Config::getInstance().field.FIELD_MARIGIN );
    double minXvalue ( Config::getInstance().field.FIELD_BOTTOM_LEFT_CORNER.x + Config::getInstance().field.FIELD_MARIGIN );
    double maxYvalue ( Config::getInstance().field.FIELD_TOP_RIGHT_CORNER.y - Config::getInstance().field.FIELD_MARIGIN );
    double minYvalue ( Config::getInstance().field.FIELD_BOTTOM_LEFT_CORNER.y + Config::getInstance().field.FIELD_MARIGIN );

    double obsRadious = Config::getInstance().getRRTRobotRadius()*1.2;
	double a = 2.0*sqrt(3.0)*obsRadious; //bok trojkata rownobocznego w ktory wpisany jest okrag opisujacy przeszkode

	double h = ( a*sqrt(3.0) )/2.0;

	/*
	*
	* 	  c3|\
	* 		| \
	* 		|  \
	* 	    |  /c2
	* 		| /
	* 	  c1|/
	*
	*/

	//znajdz trojkat rownoboczny w ktory wpisany jest okrag o promieniu przeszkody i srodku w p1
	Vector2D control_points[6];
	control_points[0] = Vector2D( this->p1.x - 0.5*a, this->p1.y + (1.0/3.0)*h );
	control_points[1] = Vector2D( this->p1.x , this->p1.y - (2.0/3.0)*h );
	control_points[2] = Vector2D( this->p1.x + 0.5*a, this->p1.y + (1.0/3.0)*h );

	/*
	* 		    c3
	* 		  /|
	* 		 / |
	* 		/  |
	* 	 c1 \  |
	* 		 \ |
	* 		  \| c2
	*
	*/

	//znajdz trojkat rownoboczny w ktory wpisany jest okrag o promieniu przeszkody i srodku w w p2
	//Vector2D pv2[3];
	control_points[3] = Vector2D ( this->p2.x + 0.5*a, this->p2.y - (1.0/3.0)*h );
	control_points[4] = Vector2D ( this->p2.x , this->p2.y + (2.0/3.0)*h );
	control_points[5] = Vector2D ( this->p2.x - 0.5*a, this->p2.y - (1.0/3.0)*h );

	for(int i=0;i<6;i++){
		LOG_INFO(log," control_point["<<i<<"]= ("<<control_points[i].x<<","<<control_points[i].y<<" )" );
	}
	int goalPoint = 0;
	Pose goalPose = Pose(control_points[0],0);

	std::pair<double, double > xConstraint;
	std::pair<double, double > yConstraint;

	bool gettingBall = false;

    while(true){
	   taskStatus = Task::not_completed;

	   if( Videoserver::getInstance().updateGameState( gameState ) < 0)
		   throw SimulationException("FollowLineAndAvoidObs::execute");

	   //ballPosition = gameState->getBallPos().getPosition();
	   //goalPose = Pose( ballPosition.projectionOn(a,b,c), 0);

	   //odleglosc od pkt docelowego przy jakiej stwierdzamy ze robot dojechal do celu
	   double minDist = 0.1;


	   LOG_DEBUG( log,"#############change control point to "<<goalPoint<<" pose = "<<goalPose );

	   this->currentTask = TaskSharedPtr( new GoToPose( goalPose, &robot,  minDist) );
	   //bestScore = score;
	   Task* newTask;

	   //Vector2D startPostion( 2.7 , 1.0)
	   Pose currPose;
	   currPose = gameState->getRobotPos(robot.getRobotID());

	   if( goalPoint==2 || goalPoint==3 || goalPoint==4 ){
		  //std::cout<<" ####################### FollowLineAndAvoidObs add constraint goalPoint==5 || goalPoint==0 || goalPoint==1 ################# "<<std::endl;
		  double delta = 0.5;
		  xConstraint = std::pair< double, double >(
				  ( currPose.get<0>( ) < goalPose.get<0>( ) ) ?  (currPose.get<0>( ) - delta) : ( goalPose.get<0>( ) - delta ),
						  maxXvalue );
		  yConstraint = std::pair< double, double>(minYvalue, maxYvalue );
	   }
	   else if( goalPoint==5 || goalPoint==0 || goalPoint==1 ){
		   //std::cout<<" ####################### FollowLineAndAvoidObs add constraint goalPoint==5 || goalPoint==0 || goalPoint==1################# "<<std::endl;
		   double delta = 0.5;
		   xConstraint = std::pair< double, double >(
				   minXvalue,
						   ( currPose.get<0>( ) > goalPose.get<0>( ) ) ?  ( currPose.get<0>( ) + delta) : ( goalPose.get<0>( ) + delta ) );
		   yConstraint = std::pair< double, double >( minYvalue, maxYvalue );

	   }
	   while(taskStatus!=Task::ok /* && !gettingBall */ ){

			if( Videoserver::getInstance().updateGameState( gameState ) < 0)
				   throw SimulationException("FollowLineAndAvoidObs::execute");

			EvaluationModule::ballState bs = evaluation.getBallState(  this->robot.getRobotID( ) );

			LOG_FATAL( log,"Ball state is  "<<bs );

			newTask = new GoToPose( goalPose, &robot,  minDist);
			this->currentTask = TaskSharedPtr(newTask );

			/*
			//jesli pilka jest wolna to zdobadz ja
			if( bs == EvaluationModule::free ){
			   newTask = new GetBall( &this->robot );
			   gettingBall = true;
			   this->currentTask = TaskSharedPtr(newTask );
			}
			//jesli jestes w posiadaniu pilki to jedz do celu
			else if( bs == EvaluationModule::mine ){
				gettingBall = false;
				newTask = new GoToPose( goalPose, &robot,  minDist);
				this->currentTask = TaskSharedPtr(newTask );
			}
			//jesli pilka jest zajeta to jedz do celu
			else{
			   gettingBall = false;
			   newTask = new GoToPose( goalPose, &robot,  minDist);
			   this->currentTask = TaskSharedPtr(newTask );
			}*/

			newTask = this->currentTask->nextTask();

			if( newTask ){
			   this->currentTask = TaskSharedPtr( newTask );
			}

			GoToPose * t;
			if( ( t = dynamic_cast<GoToPose * >( this->currentTask.get() ) )  ){
			   t->addXConstraint( &xConstraint );
			   t->addYConstraint( &yConstraint );
			}

			int steps=1;
			taskStatus = this->currentTask->execute(NULL,steps);

			if( taskStatus == Task::error ){
				LOG_FATAL(log,"FollowLineAndAvoidObs Task::error ");
				break;
			}

			if( taskStatus == Task::collision ){
				robot.stop();
				LOG_FATAL(log,"FollowLineAndAvoidObs Task::collision ");
				return;
			}
	   }
	   goalPoint ++;
	   if( goalPoint > 5 )
		   goalPoint = 0;

	   //goalPose = control_points[goalPoint++];

	   goalPose = Pose( control_points[goalPoint], 0 );

	}
}

bool FollowLineAndAvoidObs::isFinish(){

	return false;
}

FollowLineAndAvoidObs::~FollowLineAndAvoidObs() {
	// TODO Auto-generated destructor stub
}

