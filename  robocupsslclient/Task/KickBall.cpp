/*
 * KickBall.cpp
 *
 *  Created on: 2010-06-07
 *      Author: maciek
 */

#include "KickBall.h"

KickBall::KickBall(Robot * robot, double rotation_): Task(robot), rotation(rotation_) {


}


double calculateAngularVel2(GameState & gameState,std::string robotName, Pose targetPosition){
    //static GameState oldGameState;
    static double oldTetaCel;
    double rotacjaDocelowa=atan2(targetPosition.get<0>(),targetPosition.get<2>());
    double Ker=0.5;
    double Ko=20;
    double currGlobalRobotRot=gameState.getRobotPos( robotName ).get<2>();
    //macierz obrotu os OY na wprost robota
    RotationMatrix rmY(currGlobalRobotRot);
    //macierz obrotu os OY nw wprost robota
    //RotationMatrix rmY(-M_PI/2);
    //pozycja robota w ukladzie wsp zw z plansza
    //Vector2D currentPosition=Videoserver::data.getPosition(this->robotName);
    Pose currRobotPose=gameState.getRobotPos( robotName );
    //pozycja celu w ukladzie wsp zwiazanych z robotem
    Pose reltargetPose=targetPosition.transform(currRobotPose.getPosition(),rmY);
    //targetPosition=rmX.Inverse()*(goToPosition-currentPosition);
    //rotacja do celu
    double currTetaCel=atan2( (reltargetPose.get<1>()) , (reltargetPose.get<0>()));

    //double currTetaCel=atan( (-reltargetPose.get<1>()) / (reltargetPose.get<0>()));

    double angularVel=Ko*(rotacjaDocelowa-currGlobalRobotRot)+ Ker*(oldTetaCel-currTetaCel);

    oldTetaCel=currTetaCel;

    return angularVel;
}

bool KickBall::run(void * arg, int steps ){

    GameStatePtr currGameState( new GameState() );
    double currSimTime=video.updateGameState(currGameState);
	double lastSimTime=0;

    Pose goalPose=(*currGameState).getRobotPos( robot->getRobotName() );
    goalPose.get<2>()=rotation;
    while(!this->stopTask && (steps--)!=0 ){
    	if( lastSimTime < ( currSimTime=video.updateGameState(currGameState) ) ){
			lastSimTime=currSimTime;
			robot->setRelativeSpeed( Vector2D(0.0,0.0),
                            calculateAngularVel2(*currGameState,robot->getRobotName(), goalPose) );
    	}

    }
	this->robot->kick();
	return true;
}

KickBall::~KickBall() {

}
