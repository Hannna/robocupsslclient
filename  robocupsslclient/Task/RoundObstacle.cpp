/*
 * RoundObstacle.cpp
 *
 *  Created on: Oct 3, 2011
 *      Author: maciek
 */
#include <boost/math/special_functions/factorials.hpp>
#include <boost/math/special_functions/binomial.hpp>

#include "RoundObstacle.h"
#include "GoToPose.h"
#include "../Vector2d/Vector2D.h"

#include <iostream>
#include <fstream>


RoundObstacle::RoundObstacle(Robot * robot, const Vector2D obstacleCoordinates_, const double obstacleRadiuous ):
	Task(robot),obstacleCoordinates( obstacleCoordinates_ ), obstacle_r( obstacleRadiuous )
{
	LOG_INFO(log, "robot "<<robot->getRobotName()<<" create RoundObstacle Task, obstacleCoordinates "<<this->obstacleCoordinates );
}

Task* RoundObstacle::nextTask(){

	return NULL;
}

/*
double bernstein(double n, double i, double t){
	if( (i>0) && (i<n) && (t>=0) && (t<=1) ){
		return boost::math::binomial_coefficient<double>( n, i)*pow(t,i)*pow((1-t),n-i);
	}
	else
		return 0;

}
*/

/*
 * @param n stopien krzywej beziera
 * @param t punkt w ktorym obliczamy wartosc krzywej
 *
 */
/*
Vector2D RoundObstacle::bezierCurve(double n,Vector2D bezierParams[], double t){

	Vector2D result(0,0);
	for(int i=0; i<n; i++){
		double b = bernstein(n, i, t);
		result.x +=bezierParams[i].x*b;
		result.y +=bezierParams[i].y*b;
	}
	return result;
}
*/

Task::status RoundObstacle::run(void * arg, int steps){

	//initBezierParams(const Vector2D a,const  Vector2D b,const  Vector2D c,const  Vector2D d);

	Vector2D goalPose;
	//while( !this->stopTask && (steps--)!=0 ){
	while( true ){
		video.updateGameState(currGameState);
		Pose currRobotPose = (*currGameState).getRobotPos( robot->getRobotID() );

		//wyznacz rownanie parametryczne okregu po jakim ma poruszac sie robot
		double x0 = obstacleCoordinates.x;
		double y0 = obstacleCoordinates.y;
		double r = obstacle_r;
		Vector2D ox(1.0,0.0);
		Vector2D v = Vector2D( currRobotPose.get<0>() - x0, currRobotPose.get<1>() - y0 );
		double alfa = v.angleTo( ox );

		std::string file_v_name( robot->getRobotName() +"round" );
		std::ofstream file_v( file_v_name.c_str() , ios_base::in | ios_base::trunc );

		const double minDist = 0.002;

		double maxW =M_PI;
		boost::tuple< double, double, double > vel = calculateCurwatureVelocity( 0.02, maxW );
		Vector2D robotNewGlobalVel = Vector2D( vel.get<0>(), vel.get<1>() );
		double w = vel.get<2>();
		robot->setRelativeSpeed( robotNewGlobalVel, w );
		//robot->setGlobalSpeed( robotNewGlobalVel, w, currRobotPose.get<2>() );
		//sleep(60);
	}/*
		for(;alfa<2*M_PI;alfa+=0.1){
			video.updateGameState(currGameState);
			currRobotPose = (*currGameState).getRobotPos( robot->getRobotID() );

			goalPose.x = x0 +r*cos(alfa);
			goalPose.y = y0 +r*sin(alfa);

	    	Vector2D v( x0 - currRobotPose.get<0>()  , y0 - currRobotPose.get<1>()  );
	    	//idealna rotacja robota do celu
	    	Vector2D oy(0.0,1.0);
	    	double tetad = 0;
	    	tetad = oy.angleTo( v );



			TaskSharedPtr newTask = TaskSharedPtr( new GoToPose( goalPose, tetad,robot,  minDist) );
			Task::status taskStatus = Task::not_completed;

			while(taskStatus!=Task::ok){
				taskStatus = newTask->execute(NULL,1);
				LOG_INFO( log,"GoToPose status "<<taskStatus );
			}
			file_v<<goalPose.x<<";"<<goalPose.y<<std::endl;
		    file_v.flush();
		}
		file_v.close();
		//tetad = v.angleTo( oy );
	}
	*/
/*
	if( this->obstacleCoordinates.distance(currGameState->getRobotPos( this->robot->getRobotID() ).getPosition() ) < 1.0 ){
		goalPose = this->obstacleCoordinates - Vector2D( 0.0 , 2.0*Config::getInstance().getRRTRobotRadius()*sqrt(3.0) );

		/*
		if( this->d == RoundObstacle::LEFT){
			goalPose = this->obstacleCoordinates ;
		}
		else if( this->d == RoundObstacle::RIGHT ){

		}
		*
	}

	for(;t<1.0;t+=0.1){
		if( (curr_steps--) ==0 ){
			return Task::not_completed;
		}
		double n=5;
		goalPose = this->bezierCurve(n,this->bezierParams,t);
		this->goToPose =  new GoToPose( Pose(goalPose,0), robot );
		return this->goToPose->execute(arg,1);
	}
*/
	return Task::ok;
}

/*
void RoundObstacle::initBezierParams(const Vector2D a,const Vector2D b,const Vector2D c,const Vector2D d){
	bezierParams[0] = a;
	bezierParams[1] = b;
	bezierParams[2] = c;
	bezierParams[3] = d;
}
*/
/*
Vector2D RoundObstacle::bezierCurve(const double t){
	assert(t>0);
	assert(t<1);

	double x = bezierParams[0].x*pow((1-t),3) + 3*bezierParams[1].x*pow((1-t),2) +
			3*bezierParams[2].x*pow(t,2) + bezierParams[2].x*pow(t,3);
	double y = bezierParams[0].y*pow((1-t),3) + 3*bezierParams[1].y*pow((1-t),2) +
			3*bezierParams[2].y*pow(t,2) + bezierParams[2].y*pow(t,3);

	return Vector2D(x,y);
}
*/

RoundObstacle::~RoundObstacle() {

}
