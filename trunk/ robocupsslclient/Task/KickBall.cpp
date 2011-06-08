/*
 * KickBall.cpp
 *
 *  Created on: 2010-06-07
 *      Author: maciek
 */

#include "KickBall.h"
#include "GoToBall.h"


KickBall::KickBall(Robot * robot_, double rotation_): Task(robot_), kickNow(false), rotation(rotation_) {
	LOG_INFO(log," Create KickBall task. Angle to shoot "<<rotation_);

}

KickBall::KickBall(Robot * robot_): Task(robot_), kickNow(true),rotation(0) {
	LOG_INFO(log," Create KickBall task. Shoot now");

}

Task* KickBall::nextTask(){

	//jesli pilka jest za daleko to podjedz do pilku

	if( !this->evaluationModule.isRobotOwnedBall( this->robot ) ){
		LOG_INFO(this->log,"MoveBall -> GoToBall");
		//jesli robot nie ma pilki to zmieni task na GoToBall
		return new GoToBall( this->robot );
	}
	else
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
    double currTetaCel= convertAnglePI( goalRotation - currRobotPose.get<2>() );
    double angularVel=Ko*(currTetaCel) + Ker*(oldTetaCel-currTetaCel);

    oldTetaCel=currTetaCel;

    return fabs( angularVel ) > M_PI/2 ? M_PI/2 * sgn(angularVel) : angularVel;
    //return angularVel;
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

    while( !this->kickNow &&  !this->stopTask && (steps--)!=0 ){
    	if( lastSimTime < ( currSimTime=video.updateGameState(currGameState) ) ){
			currPose = (*currGameState).getRobotPos( robot->getRobotID() );
			lastSimTime=currSimTime;

			if( task_status == Task::ok){
				if( fabs( currGameState->getRobotAngularVelocity( robot->getRobotID() ) )  < 0.1 ){
					this->stopTask=true;
				}
			}

            if(  ( error=pow( rotation - currPose.get<2>(),2 )  )   < ROTATION_PRECISION ){
                //this->stopTask=true;
                robot->setRelativeSpeed( Vector2D(0.0,0.0), 0 );
                task_status = Task::ok;
            }
            else{
    			double w = calculateAngularVel2( currPose , rotation);
    			LOG_INFO(log,"shoot rotation "<<rotation<<" robot rotation "<<currPose.get<2>()<<"set speed w "<< w);
    			robot->setRelativeSpeed( Vector2D(0.0,0.0), w );
            }
    	}
    }
    LOG_INFO(log,"Have good position. Try to kick ball.");

    if( task_status == Task::ok || this->kickNow ){
    	this->robot->kick();
		return Task::kick_ok;
    }
    else
    	return Task::not_completed;
}

KickBall::~KickBall() {

}
