/*
 * KickBall.cpp
 *
 *  Created on: 2010-06-07
 *      Author: maciek
 */

#include "KickBall.h"
#include "GoToBall.h"


KickBall::KickBall( Robot * robot_, Vector2D  targetPosition_, double rotation_, double maxForce ): Task(robot_), kickNow(false), rotation( rotation_ ), targetPosition(targetPosition_),
maxKickForce( maxForce ) {
	LOG_INFO(log," Create KickBall task. Angle to shoot "<<rotation<<" position to shoot "<<targetPosition);

}
/*
KickBall::KickBall(Robot * robot_): Task(robot_), kickNow(true),rotation(0) {
	LOG_INFO(log," Create KickBall task. Shoot now");

}
*/
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


double KickBall::calculateAngularVel2( const Pose & currRobotPose, const double goalRotation ){

    static double oldTetaCel;

    double Ker=0.5;
    double Ko=20;

    double sgn=1;

    if( fabs( goalRotation - currRobotPose.get<2>() ) < M_PI ){
    	if( goalRotation > currRobotPose.get<2>() ){
    		sgn = 1;
    	}
    	else
    		sgn = -1;
    }
	else{
	   	if( goalRotation > currRobotPose.get<2>() ){
	    	sgn = -1;
	    }
	    else
	    	sgn = 1;
	}


    //rotacja do celu
    double currTetaCel= convertAnglePI( goalRotation - currRobotPose.get<2>() );
    double angularVel=Ko*(currTetaCel) + Ker*(oldTetaCel-currTetaCel);

    oldTetaCel=currTetaCel;

    return fabs( angularVel ) > M_PI ? M_PI * sgn : fabs(angularVel)*sgn;
    //return angularVel;
}


Task::status KickBall::run(void * arg, int steps ){

    LOG_DEBUG(log,"starting KickBall task");
    GameStatePtr currGameState( new GameState() );
    double currSimTime=video.updateGameState(currGameState);
	double lastSimTime=0;

    Pose currPose;
    double error;
    Task::status task_status = Task::not_completed;

    while( !this->kickNow &&  !this->stopTask && (steps--)!=0 ){
    	if( lastSimTime < ( currSimTime=video.updateGameState(currGameState) ) ){
			currPose = (*currGameState).getRobotPos( robot->getRobotID() );
			lastSimTime=currSimTime;

			if( task_status == Task::ok) {
				if( fabs( currGameState->getRobotAngularVelocity( robot->getRobotID() ) )  < 0.1 ){
					this->stopTask=true;
				}
			}

			//oblicz blad wzgledny
			//error= fabs(  (currPose.get<2>() - rotation ) / rotation )*100.0;
			error= fabs(  (currPose.get<2>() - rotation ) );
            //if(  ( error=pow( rotation - currPose.get<2>(),2 )  )   < ROTATION_PRECISION ){
			double threshold = 0.017;
			if(this->predicates & Task::pass){
				threshold = 0.017 * currPose.getPosition().distance(this->targetPosition);
			}
			if( error < threshold ){//1stopien
                robot->setRelativeSpeed( Vector2D(0.0,0.0), 0 );
                task_status = Task::ok;
                LOG_INFO(log,"Task::ok shoot rotation "<<rotation<<" robot rotation "<<currPose.get<2>() );


            }
            else{
    			double w = calculateAngularVel2( currPose , rotation);
    			LOG_INFO(log,"shoot rotation "<<rotation<<" robot rotation "<<currPose.get<2>()<<"set speed w "<< w<<" current error "<<error);
    			LOG_INFO(log,"current robot ang speed "<<currGameState->getRobotAngularVelocity( robot->getRobotID() )<<" linear vel "<<currGameState->getRobotGlobalVelocity(robot->getRobotID() )  );
    			robot->setRelativeSpeed( Vector2D(0.0,0.0), w );
    			task_status = Task::not_completed;
            }
    	}
    }

    if( task_status == Task::ok  || this->kickNow  ){
    	if(task_status == Task::ok){
    		LOG_INFO(log,"shoot rotation "<<rotation<<" robot rotation "<<currPose.get<2>());
    		LOG_INFO(log,"Have good position. Try to kick ball.");
    	}
    	else if( this->kickNow ){
    		LOG_INFO(log,"kick now set. Try to kick ball.");
    	}
    	//double maxForce = 5.0;
    	double dist = currPose.getPosition().distance( this->targetPosition );
    	LOG_INFO(log,"dist to target "<< dist);
    	double force = dist*maxKickForce/9.16;
    	this->robot->kick( force );

    	if( this->evaluationModule.isRobotOwnedBall( this->robot->getRobotID() ) ){
     		LOG_INFO(log,"Robot still have ball");
    		return Task::not_completed;
    	}
    	else{

    		if(this->predicates & Task::kick_for_dribble){
    			LOG_INFO(log,"Was ball kicked for dribble ");
    			//exit(0);
    			return Task::ok;
    		}

    		LOG_INFO(log,"Was ball kicked");
    		return Task::kick_ok;
    	}
    }
    else
    	return Task::not_completed;
}

KickBall::~KickBall() {

}
