/*
 * KickBall.cpp
 *
 *  Created on: 2010-06-07
 *      Author: maciek
 */

#include "KickBall.h"

KickBall::KickBall(Robot * robot_, double rotation_): Task(robot_), rotation(rotation_) {
	LOG_INFO(log," Create KickBall task. Angle to shoot "<<rotation_);

}

Task* KickBall::nextTask(){
	return NULL;
}
/*
double Robot::calculateAngularVel(const Pose & currRobotPose, const double goalRotation){

	//obrot jaki trzeba było wykonac w poprzednim kroku
	static double oldAlfaToCel;

    double Ker=0.5;
    double Ko=20;
    double currGlobalRobotRot=gameState.getRobotPos( robotID ).get<2>();
    //pozycja robota w ukladzie wsp zw z plansza
    Pose currRobotPose=gameState.getRobotPos( robotID );


    RotationMatrix rm0(0);
    Pose reltargetPose = globalTargetPosition.transform(currRobotPose.getPosition(),rm0);
    double rotacjaDocelowa=-atan2(reltargetPose.get<0>(),reltargetPose.get<1>()) ;// (M_PI/2);

    //macierz obrotu os OY na wprost robota
    //RotationMatrix rmY(currGlobalRobotRot);
    //macierz obrotu os OY nw wprost robota
    //RotationMatrix rmY(-M_PI/2);
    //pozycja celu w ukladzie wsp zwiazanych z robotem
    //Pose reltargetPose=globalTargetPosition.transform(currRobotPose.getPosition(),rmY);
    //targetPosition=rmX.Inverse()*(goToPosition-currentPosition);

    //obrot jaki trzeba wykonac w biezacym kroku
    double currAlfaToCel = rotacjaDocelowa - currGlobalRobotRot;

    //double currTetaCel=atan( (-reltargetPose.get<1>()) / (reltargetPose.get<0>()));
    // double angularVel= Ko*(blad rotacji) + Ker(obrotdo celu jaki trzeba bylo wykonac w poprzednm kroku - obrot do celu jaki trzeba wykonac w tymkroku)
    double angularVel=Ko*(rotacjaDocelowa-currGlobalRobotRot)+ Ker*(oldAlfaToCel - currAlfaToCel);

    oldAlfaToCel=currAlfaToCel;

    double w = fabs(angularVel) > 4*M_PI ? 4*M_PI * sgn(angularVel) : angularVel;
    LOG_INFO(log,"rotacjaDocelowa "<<currAlfaToCel<<" rotacjaDocelowa "<<rotacjaDocelowa<<" calculate angular vel w="<< w);
    return  w;
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
    Task::status task_status = Task::not_completed;
    while(!this->stopTask && (steps--)!=0 ){
    	if( lastSimTime < ( currSimTime=video.updateGameState(currGameState) ) ){
			currPose = (*currGameState).getRobotPos( robot->getRobotID() );
			lastSimTime=currSimTime;
			double w = calculateAngularVel2( currPose , rotation);
			robot->setRelativeSpeed( Vector2D(0.0,0.0), w );

            //LOG_DEBUG(log,"#####################################################");
            //LOG_DEBUG(log,"current error "<<error<<" set angular vel "<<w);

            //Pose currRobotPose=(*currGameState).getRobotPos( robot->getRobotName() );
            if(  ( error=pow( rotation - currPose.get<2>(),2 )  )   < ROTATION_PRECISION ){
                this->stopTask=true;
                robot->setRelativeSpeed( Vector2D(0.0,0.0), 0 );
                task_status = Task::ok;
            }
    	}
    }
    LOG_DEBUG(log,"Have good position. Try to kick ball.");

    ///if()
    if(task_status == Task::ok){
    	this->robot->kick();
		return Task::ok;
    }
    else
    	return Task::not_completed;
}

KickBall::~KickBall() {

}
