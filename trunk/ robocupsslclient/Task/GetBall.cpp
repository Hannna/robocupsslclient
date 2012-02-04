/*
 * GetBall.cpp
 *
 *  Created on: 24-04-2011
 *      Author: maciek
 */

#include "GetBall.h"
#include "GoToBall.h"
#include "../Exceptions/SimulationException.h"

//ios_base::trunc

GetBall::GetBall(Robot * robot):Task(robot),file_name( robot->getRobotName() + "getBall" ), file( file_name.c_str( ), ios_base::in | ios_base::app ) {
	// TODO Auto-generated constructor stub
	LOG_INFO(log, "robot "<<robot->getRobotName()<<" create GetBall Task" );
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
		LOG_INFO(log, "change GetBall ->> GoToBall distance to ball "<<toBall.length() );
		return new GoToBall(this->robot);
	}
	return NULL;
}

Task::status GetBall::run(void * arg, int steps){

	GameStatePtr currGameState(new GameState());
	Pose currRobotPose;
	Pose ballPose;
	Vector2D ballPos;

	Vector2D robotCurrentVel;
	double lastSimTime = 0;
	double currSimTime = 0;

	double distanceToBall=0;
	double angleToBall=0;
	bool hasBall=false;

	while( !this->stopTask && (steps--)!=0 ){

		if( Videoserver::getInstance().updateGameState( currGameState ) < 0 )
		    throw SimulationException("GetBall::run");

		if( lastSimTime < ( currSimTime=video.updateGameState(currGameState) ) ){
			hasBall = this->evaluationModule.isRobotOwnedBall( *(this->robot) ,currGameState, distanceToBall,angleToBall);
			if(hasBall){
				LOG_INFO(log, "GetBall, robot "<<robot->getRobotName()<<" have ball" );
				break;
			}
			if( distanceToBall > this->maxDistanceToBall ){
				LOG_INFO(log, "GetBall, ball is too far. Distance to ball "<<distanceToBall );
				return Task::not_completed;
			}

			currRobotPose = currGameState->getRobotPos( this->robot->getRobotID() );
			ballPose = currGameState->getBallPos();
			//ballPos = ballPose.getPosition();

			//ballPos = this->currGameState->getBallPos().getPosition();


			//jedz do pilki
			//robot->goToBall( this->maxDistanceToBall );
			if( strcmp( this->robot->getRobotName().c_str(), "blue0" ) == 0 ){
				double reference = 0;
				file<<reference<<";"<<angleToBall<<";"<<std::endl;
			}
			lastSimTime = currSimTime;
			//czy pilka jest na wprost robota
			if ( fabs(angleToBall) < 0.33 ){
				//czy jest odpowiednia odleglosc
				if( distanceToBall < 0.01 ){
					//LOG_INFO(log, "robotPosition "<<currRobotPose<<" ball position "<< ballPose<<" angleToBall "<<angle<<" distance to ball "<< toBall.length());
					LOG_INFO(log, "GetBall Task::ok" );
					return Task::get_ball;
				}//podjedz do pilki
				else{
					ballPos = ballPose.getPosition();
					LOG_INFO(log, "GetBall ballPos wo prediction "<<ballPos<<" ball v "<<currGameState->getBallGlobalVelocity()  );
					Vector2D t =ballPos +  ( currGameState->getBallGlobalVelocity() * ( 10.0* Videoserver::getInstance().getUpdateDeltaTime( ) ) );
					LOG_INFO(log, "GetBall ballPos after prediction "<<t );
					bool haveBall=true;
					Vector2D robotNewVel=this->robot->calculateVelocity( robotCurrentVel, currRobotPose, Pose( t,0 ),haveBall );
					robot->setGlobalSpeed( robotNewVel, 0, currRobotPose.get<2>() );
					LOG_INFO(log, "GetBall podjedz do pilki o "<<distanceToBall );
					return Task::not_completed;
				}
			}
			//obroc robota
			else{
				LOG_INFO(log, "GetBall obroc robota do pilki o "<<angleToBall );
				double oldAlfaToCel = robot->getLastTetaToBall();

				double Ker=0.5;
				double Ko=2.5;
				double currGlobalRobotRot = currRobotPose.get<2>();

				// ten kawalek kodu wyznacza kat o jaki robot musi sie obrocic zeby byc skierowanym na cel

				//Pose ballPose( ballPos,0 );
				ballPose = Pose(ballPose.getPosition() + ( this->currGameState->getBallGlobalVelocity() * 2.0* Videoserver::getInstance().getUpdateDeltaTime( ) ),0);
				//ballPos = ballPos + ( this->currGameState->getBallGlobalVelocity() * 10.0* Videoserver::getInstance().getUpdateDeltaTime( ) );

				ballPos = ballPose.getPosition();

				double rotacjaDocelowa = calculateProperAngleToTarget(currRobotPose,ballPose );
				//RotationMatrix rm0(0);
				//Pose reltargetPose_ = ballPose.transform( currRobotPose.getPosition(),rm0 );
				//Pose reltargetPose = reltargetPose_*100;
				//double rotacjaDocelowa=-atan2(reltargetPose.get<0>(),reltargetPose.get<1>()) ;

				assert( fabs(rotacjaDocelowa) < M_PI);

				//obrot jaki trzeba wykonac w biezacym kroku
				//double currAlfaToCel = convertAnglePI( rotacjaDocelowa - currGlobalRobotRot );

				double currAlfaToCel=calculateAngleToTarget(currRobotPose, ballPose );

				double angularVel=Ko*( convertAnglePI(rotacjaDocelowa-currGlobalRobotRot) )+ Ker*( convertAnglePI(oldAlfaToCel - currAlfaToCel) );

				oldAlfaToCel=currAlfaToCel;
				robot->setLastTetaToBall(currAlfaToCel);
				double w = fabs(angularVel) > M_PI ? M_PI * sgn(angularVel) : angularVel;

				//podjedz tez do pilki
				if( distanceToBall > 0.02 ){
					Pose ballPose ( ballPos + ( this->currGameState->getBallGlobalVelocity() * 10.0* Videoserver::getInstance().getUpdateDeltaTime( ) ),0 );
					LOG_INFO(log, "robotPosition "<<currRobotPose<<" ballPos "<<ballPos<<" with prediction ball position "<< ballPose);

					Vector2D robotNewVel=this->robot->calculateVelocity( robotCurrentVel, currRobotPose, ballPose );
					robot->setGlobalSpeed( robotNewVel, w, currRobotPose.get<2>() );
					LOG_INFO(log, "robotPosition "<<currRobotPose<<" ball position "<< ballPose);
					LOG_INFO(log," obrot robota do pilki o currAlfaToCel "<<currAlfaToCel<<"  set w "<< w<<" i dojazd do pilki speed "<<robotNewVel  );

				}
				else{
					robot->setRelativeSpeed( Vector2D(0,0), w );
					LOG_INFO(log, "robotPosition "<<currRobotPose<<" ball position "<< ballPose <<" obrot robota do pilki o currAlfaToCel "<<currAlfaToCel<<" set w "<< w );
				}
				return Task::not_completed;
			}
		}
	}
	return Task::ok;
}

GetBall::~GetBall() {
	// TODO Auto-generated destructor stub
	this->file.close();
}
