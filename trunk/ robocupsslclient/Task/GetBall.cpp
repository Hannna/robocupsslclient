/*
 * GetBall.cpp
 *
 *  Created on: 24-04-2011
 *      Author: maciek
 */

#include "GetBall.h"
#include "GoToBall.h"
#include "../Exceptions/SimulationException.h"

GetBall::GetBall(Robot * robot):Task(robot) {
	// TODO Auto-generated constructor stub

}

Task* GetBall::nextTask(){

	GameStatePtr currGameState(new GameState());

	if( Videoserver::getInstance().updateGameState( currGameState ) < 0 )
	    throw SimulationException("GetBall::nextTask()");

	//video.updateGameState(currGameState);

	Pose robotPos = currGameState->getRobotPos(this->robot->getRobotID() );
	Pose ballPos = currGameState->getBallPos();
	Vector2D toBall = Vector2D( ballPos.getPosition() - robotPos.getPosition() );

	if( toBall.length() > this->maxDistanceToBall ){
		LOG_INFO(log, "change GetBall ->> GoToBall " );
		return new GoToBall(this->robot);
	}
	return NULL;
}

Task::status GetBall::run(void * arg, int steps){

	GameStatePtr currGameState(new GameState());
	Pose currRobotPose;
	Pose ballPose;
	Vector2D ballPos;

	Vector2D reference;
	Vector2D toBall;
	double angle;
	Vector2D robotCurrentVel;
	double lastSimTime = 0;
	double currSimTime = 0;

	while( !this->stopTask && (steps--)!=0 ){

		//video.updateGameState(currGameState);
		if( Videoserver::getInstance().updateGameState( currGameState ) < 0 )
		    throw SimulationException("GetBall::run");


		if( lastSimTime < ( currSimTime=video.updateGameState(currGameState) ) ){
			lastSimTime = currSimTime;

			robotCurrentVel = currGameState->getRobotGlobalVelocity(this->robot->getRobotID());

			//bool ballIsOwned =  false;
			currRobotPose = currGameState->getRobotPos(this->robot->getRobotID() );
			ballPose = currGameState->getBallPos();
			ballPos = ballPose.getPosition();
			ballPos = ballPos + currGameState->getBallGlobalVelocity()*video.getUpdateDeltaTime();

			toBall = Vector2D( ballPos - currRobotPose.getPosition() );
			if( toBall.length() > this->maxDistanceToBall ){
				LOG_INFO(log, "GetBall, ball is too far " );
				return Task::not_completed;
			}

			reference = Vector2D( cos( currRobotPose.get<2>()+M_PI_2 ), sin( currRobotPose.get<2>()+M_PI_2 ) );

			angle = toBall.angleTo(reference);

			//czy pilka jest przed dribblerem
			if ( fabs(angle) < 0.33 ){
				//czy pilka jest odpowienio blisko dribblera
				//0.075 srodek dribblera 0.22 troche powiekszony promien pilki
				//0.006 promien dribblera
				if ( toBall.length() < ( 0.075+0.006+0.020 ) / cos(angle) ){
					LOG_INFO(log, "GetBall Task::ok  toBall.length()"<<toBall.length() );
					return Task::get_ball;
				}//podjedz do pilki
				else{
					Vector2D robotNewVel=calculateVelocity( robotCurrentVel, currRobotPose, Pose( ballPos,0 ) );
					//robot->setRelativeSpeed( robotNewVel, 0 );
					robot->setGlobalSpeed( robotNewVel, 0, currRobotPose.get<2>() );
					///LOG_INFO(log, "GetBall podjedz do pilki " );
					return Task::not_completed;
				}
			}//obroc robota
			else{

				//obrot jaki trzeba było wykonac w poprzednim kroku
				static double oldAlfaToCel;

				double Ker=0.5;
				double Ko=20;
				double currGlobalRobotRot = currRobotPose.get<2>();

				// ten kawalek kodu wyznacza kat o jaki robot musi sie obrocic zeby byc skierowanym na cel
				RotationMatrix rm0(0);
				Pose ballPose( ballPos,0 );

				Pose reltargetPose_ = ballPose.transform( currRobotPose.getPosition(),rm0 );
				Pose reltargetPose = reltargetPose_*100;
				double rotacjaDocelowa=-atan2(reltargetPose.get<0>(),reltargetPose.get<1>()) ;

				assert( fabs(rotacjaDocelowa) < M_PI);

				//obrot jaki trzeba wykonac w biezacym kroku
				double currAlfaToCel = convertAnglePI( rotacjaDocelowa - currGlobalRobotRot );

				double angularVel=Ko*( convertAnglePI(rotacjaDocelowa-currGlobalRobotRot) )+ Ker*( convertAnglePI(oldAlfaToCel - currAlfaToCel) );

				oldAlfaToCel=currAlfaToCel;

				double w = fabs(angularVel) > M_PI/2 ? M_PI/2 * sgn(angularVel) : angularVel;


				robot->setRelativeSpeed( Vector2D(0,0), w );
				LOG_INFO(log, "obrot robota do pilki  currAlfaToCel "<<currAlfaToCel<<" w"<<w );
				//usleep(10000);
				return Task::not_completed;
			}
		}
	}

	return Task::ok;
}

GetBall::~GetBall() {
	// TODO Auto-generated destructor stub
}
