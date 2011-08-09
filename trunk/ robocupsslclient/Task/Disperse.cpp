/*
 * Disperse.cpp
 *
 *  Created on: Aug 9, 2011
 *      Author: maciek
 */

#include "Disperse.h"
#include <boost/foreach.hpp>
#include <boost/bind.hpp>
//#include <boost\math\special_functions\fpclassify.hpp>
#include <gsl/gsl_deriv.h>


/*
 * GetBall.cpp
 *
 *  Created on: 24-04-2011
 *      Author: maciek
 */

#include "GetBall.h"
#include "GoToBall.h"
#include "../Exceptions/SimulationException.h"

Disperse::Disperse(Robot * robot):Task(robot) {
	// TODO Auto-generated constructor stub

}

Task* Disperse::nextTask(){

	return NULL;
}

Task::status Disperse::run(void * arg, int steps){

	GameStatePtr currGameState(new GameState());
	Pose currRobotPose;
	Pose ballPose;
	Vector2D ballPos;

	Vector2D reference;
	Vector2D toBall;
	Vector2D robotCurrentVel;

	std::list< Vector2D >obstacles;
	double distToObstacle = numeric_limits<double>::infinity();

	while( ( !this->stopTask && (steps--)!=0 ) || distToObstacle < this->minimalDistanceToObstacle ){

		//odsuwa robota od przeszkod za pomoca metody pol potencjalowych
		//konstruuje pole ujemne
		LOG_INFO(log,"start  disperse for robot "<<this->robot->getRobotName()<<" minimal distance from obstacle "<<this->minimalDistanceToObstacle );

		double lastTime = 0;
		double currTime = 0;

		distToObstacle = numeric_limits<double>::infinity();

		GameStatePtr gameState( new GameState() );
		Videoserver::getInstance().updateGameState( gameState );

		Vector2D goalPose;
		if( strncmp( this->robot->getRobotName().c_str(), "red", 3 ) == 0 ){
			goalPose = Videoserver::getInstance().getRedGoalMidPosition();
			LOG_INFO(log,"red robot, goal Pose "<<goalPose);
		}else{
			goalPose = Videoserver::getInstance().getBlueGoalMidPosition();
			LOG_INFO(log,"blue robot, goal Pose "<<goalPose);
		}

		Vector2D oldGradient(0.0,0.0);
		Vector2D gradient(0.0,0.0);

		//odleglosc robota od pilki jest mniejsza niz 30 cm
		do {

			GameStatePtr gameState( new GameState() );
			if( ( currTime = Videoserver::getInstance().updateGameState( gameState ) ) > lastTime  )
			{
				distToObstacle = numeric_limits<double>::infinity();
				lastTime = currTime;
				ballPose = gameState->getBallPos();

				const std::vector<std::string> blueTeam=Config::getInstance().getBlueTeam();

				currRobotPose = gameState->getRobotPos( this->robot->getRobotID( ) );
				Pose nearestObsPose;

				BOOST_FOREACH(std::string modelName,blueTeam)
				{
					if(modelName.compare( this->robot->getRobotName() )!=0){
						obstacles.push_back( gameState->getRobotPos( Robot::getRobotID( modelName) ).getPosition() );

						if( gameState->getRobotPos( Robot::getRobotID( modelName) ).distance( currRobotPose )  <  distToObstacle ){
							distToObstacle =  gameState->getRobotPos( Robot::getRobotID( modelName) ).distance( currRobotPose );
							nearestObsPose = gameState->getRobotPos( Robot::getRobotID( modelName) );
						}

					}
				}

				const std::vector<std::string> redTeam=Config::getInstance().getRedTeam();
				BOOST_FOREACH(std::string modelName,redTeam)
				{
					if(modelName.compare( this->robot->getRobotName() )!=0){
						obstacles.push_back( gameState->getRobotPos( Robot::getRobotID(modelName) ).getPosition() );

						if( gameState->getRobotPos( Robot::getRobotID( modelName) ).distance( currRobotPose )  <  distToObstacle ){
							distToObstacle =  gameState->getRobotPos( Robot::getRobotID( modelName) ).distance( currRobotPose );
							nearestObsPose = gameState->getRobotPos( Robot::getRobotID( modelName) );
						}
					}
				}

				Vector2D g;
				gsl_function F;
				double result, abserr;
				funcParams params;
				params.obstacles = obstacles;
				params.goal = goalPose;
				params.param = currRobotPose.getPosition().y;

				F.function = &navigationFunctionX;
				F.params = &params;

				gsl_deriv_central (&F, currRobotPose.get<0>(), 1e-8, &result, &abserr);
				g.x = result;

				params.goal = goalPose;
				params.param = currRobotPose.getPosition().x;

				F.function = &navigationFunctionY;
				F.params = &params;

				gsl_deriv_central (&F, currRobotPose.get<1>(), 1e-8, &result, &abserr);
				g.y = result;

				LOG_INFO(log," gradient from lib dx="<<g.x<<" dy="<<g.y);
				LOG_INFO(log," gradient calculated dx="<<gradient.x<<" dy="<<gradient.y);

				gradient = g * (-1.0);
				oldGradient=gradient;

				obstacles.clear();

				Vector2D velocity = gradient;
				double max;
				if( fabs( velocity.x ) > fabs( velocity.y ) )
					max = fabs( velocity.x );
				else
					max = fabs( velocity.y );

				if( max > 0 )
					velocity = velocity * ( this->maxDisperseVelocity / max );

				LOG_INFO( log,"gradient "<< gradient <<"set speed "<<velocity<<" distTo nearest obs "<<distToObstacle<< " nearestObsPose"<<nearestObsPose << " dist to ball "<<ballPose.distance( currRobotPose ) );


				this->robot->setGlobalSpeed( velocity,0,currRobotPose.get<2>() );
			}

		}while( /*( ballPose.distance( currRobotPose )  < dist ) ||*/      );

		this->stop();

		LOG_INFO( log,"end  disperse for robot "<<this->robot->getRobotName() );

	}

	return Task::ok;
}

Disperse::~Disperse() {
	// TODO Auto-generated destructor stub
}

