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

RoundObstacle::RoundObstacle(Robot * robot, const Vector2D obstacleCoordinates, RoundObstacle::direction d_ ):
	Task(robot),obstacleCoordinates( obstacleCoordinates ), d(d_)
{
	t=0;
}

Task* RoundObstacle::nextTask(){

	return NULL;
}

double bernstein(double n, double i, double t){
	if( (i>0) && (i<n) && (t>=0) && (t<=1) ){
		return boost::math::binomial_coefficient<double>( n, i)*pow(t,i)*pow((1-t),n-i);
	}
	else
		return 0;

}
/*
 * @param n stopien krzywej beziera
 * @param t punkt w ktorym obliczamy wartosc krzywej
 *
 */
Vector2D RoundObstacle::bezierCurve(double n,Vector2D bezierParams[], double t){

	Vector2D result(0,0);
	for(int i=0; i<n; i++){
		double b = bernstein(n, i, t);
		result.x +=bezierParams[i].x*b;
		result.y +=bezierParams[i].y*b;
	}
	return result;
}
Task::status RoundObstacle::run(void * arg, int steps){

	//initBezierParams(const Vector2D a,const  Vector2D b,const  Vector2D c,const  Vector2D d);

	int curr_steps = 0;

	video.updateGameState(currGameState);
	Vector2D goalPose;

	if( this->obstacleCoordinates.distance(currGameState->getRobotPos( this->robot->getRobotID() ).getPosition() ) < 1.0 ){
		goalPose = this->obstacleCoordinates - Vector2D( 0.0 , 2.0*Config::getInstance().getRRTRobotRadius()*sqrt(3.0) );

		/*
		if( this->d == RoundObstacle::LEFT){
			goalPose = this->obstacleCoordinates ;
		}
		else if( this->d == RoundObstacle::RIGHT ){

		}
		*/
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

	return Task::ok;
}


void RoundObstacle::initBezierParams(const Vector2D a,const Vector2D b,const Vector2D c,const Vector2D d){
	bezierParams[0] = a;
	bezierParams[1] = b;
	bezierParams[2] = c;
	bezierParams[3] = d;
}

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
