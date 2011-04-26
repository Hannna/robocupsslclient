/*
 * KickBall.cpp
 *
 *  Created on: 2010-06-07
 *      Author: maciek
 */

#include "KickBall.h"

KickBall::KickBall(Robot * robot, double rotation_): Task(robot), rotation(rotation_) {


}

Task* KickBall::nextTask(){
	return this;
}

/*
TaskSharedPtr& KickBall::nextTask(){
	return TaskWeakPtr(this);
}
*/
double calculateAngularVel2(const Pose & currRobotPose, const double goalRotation){

    static double oldTetaCel;

    double Ker=0.5;
    double Ko=20;

    //rotacja do celu
    double currTetaCel= goalRotation - currRobotPose.get<2>();
    double angularVel=Ko*(currTetaCel) + Ker*(oldTetaCel-currTetaCel);

    oldTetaCel=currTetaCel;

    return angularVel;
}

Task::status KickBall::run(void * arg, int steps ){

    LOG_DEBUG(log,"starting KickBall task");
    GameStatePtr currGameState( new GameState() );
    double currSimTime=video.updateGameState(currGameState);
	double lastSimTime=0;

    Pose currPose;
    //double w;
    double error;

    while(!this->stopTask && (steps--)!=0 ){
    	if( lastSimTime < ( currSimTime=video.updateGameState(currGameState) ) ){
			currPose = (*currGameState).getRobotPos( robot->getRobotID() );
			lastSimTime=currSimTime;
			double w = calculateAngularVel2( currPose , rotation);
			robot->setRelativeSpeed( Vector2D(0.0,0.0), w );

            LOG_DEBUG(log,"#####################################################");
            LOG_DEBUG(log,"current error "<<error<<" set angular vel "<<w);

            //Pose currRobotPose=(*currGameState).getRobotPos( robot->getRobotName() );
            if(  ( error=pow( rotation - currPose.get<2>(),2 )  )   < ROTATION_PRECISION ){
                this->stopTask=true;
                robot->setRelativeSpeed( Vector2D(0.0,0.0), 0 );
            }
    	}
    }
    LOG_DEBUG(log,"Have good position. Try to kick ball.");
	this->robot->kick();
	return Task::ok;
}

KickBall::~KickBall() {

}
