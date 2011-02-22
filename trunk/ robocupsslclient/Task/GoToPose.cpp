/*
 * GoToPose.cpp
 *
 *  Created on: 2010-06-07
 *      Author: maciek
 */

#include "GoToPose.h"
#include "../RRT/RRTPlanner.h"

GoToPose::GoToPose(const Pose & pose,Robot * robot):Task(robot),goalPose(pose) {
	LOG_DEBUG(getLoggerPtr( (robot->getRobotName().append(".log")).c_str() ), "create GoToPose Task, goto "<<pose);

}

double calculateAngularVel(GameState & gameState,std::string robotName, Pose targetPosition){
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

bool GoToPose::run(){
	RRTPlanner * rrt;
	std::list<Pose>  path;
	GameStatePtr currGameState(new GameState());

	double currSimTime=video.updateGameState(currGameState);
	double lastSimTime=0;
	bool obsPredictionEnable=true;

	//pozycja do ktorej ma dojechac robot w kolejnym kroku
	Pose nextRobotPose;
	//biezaca pozycja robota
	Pose currRobotPose;
	//rotacja robota
	double robotRotation=0;


//	double oldToTargetAngle=0;
//	double currTotargetAngle=0;

	//pozycja celu w ukladzie wsp zwiazanych z robotem
	Vector2D targetRelPosition;

	Vector2D robotCurrentVel;
	Vector2D robotNewVel;
	/*
	 * za pomoca algorytmu rrt pokieruj robota do celu
	 */
	while(!this->stopTask ){
		if( lastSimTime < ( currSimTime=video.updateGameState(currGameState) ) ){
			lastSimTime=currSimTime;
			rrt = new RRTPlanner(Config::getInstance().getRRTGoalProb(),
						robot->getRobotName(),obsPredictionEnable,currGameState,goalPose,&path,currSimTime);

            RRTPlanner::ErrorCode status;
			status=rrt->run(video.getUpdateDeltaTime());
			if( status==RRTPlanner::Success ){

				GameStatePtr nextState=rrt->getNextState();
				if(nextState.get()==NULL){
					robot->setRelativeSpeed(Vector2D(0.0,0.0),0);
					std::cout<<"next state is null"<<std::endl;
					break;
				}
				nextRobotPose=nextState->getRobotPos(robot->getRobotName());

                std::cout<<"go to"<<nextRobotPose<<std::endl;

				robotRotation=(*currGameState).getRobotPos( robot->getRobotName()).get<2>() ;
				//macierz obrotu os OY na wprost robota
				RotationMatrix rmY(robotRotation);

				currRobotPose=(*currGameState).getRobotPos( robot->getRobotName() );

				//pozycja celu w ukladzie wsp zwiazanych z robotem
				targetRelPosition=rmY.Inverse()*(nextRobotPose.getPosition()-currRobotPose.getPosition());

                std::cout<<"go to"<<targetRelPosition<<std::endl;

				robotCurrentVel=(*currGameState).getRobotVelocity( robot->getRobotName() );
				std::cout<<"robot->robotCurrentVel"<<robotCurrentVel<<std::endl;
				robotNewVel=calculateVelocity( robotCurrentVel, Pose(targetRelPosition.x,targetRelPosition.y,0));

				std::cout<<"robot->setSpeed"<<robotNewVel<<std::endl;
				robot->setRelativeSpeed(robotNewVel,
                            calculateAngularVel(*currGameState,robot->getRobotName(), goalPose)
                );

				delete rrt;
			}
			else if(status==RRTPlanner::RobotReachGoalPose){
                return true;
			}
			else{
				robot->setRelativeSpeed(Vector2D(0.0,0.0),0);
				std::cout<<"Error nr"<<status<<std::endl;
				//exit(0);
				break;
			}
		}
	}
	/*
	 * jesli robot jest u celu to wykonaj korekte rotacji
	 */
	// double teta_cel;
	//if(!this->stopTask){
	//    do{
    //        video.updateGameState(currGameState);
    //        currRobotPose=(*currGameState).getRobotPos( robot->getRobotName() );
    //        double rot=(*currGameState).getRobotPos( robot->getRobotName()).get<2>() ;
            //macierz obrotu os OX na wprost robota
            //RotationMatrix rmX(rot);
            //macierz obrotu os OY nw wprost robota
            //RotationMatrix rmY(-M_PI/2);
            //pozycja robota w ukladzie wsp zw z plansza
            //Vector2D currentPosition=Videoserver::data.getPosition(this->robotName);

            //pozycja celu w ukladzie wsp zwiazanych z robotem
            //targetPosition=rmX.Inverse()*(goToPosition-currentPosition);
            //rotacja do celu
            //this->teta_cel=atan2( (targetPosition.y) , (targetPosition.x));
            //targetPosition = rmY.Inverse() * targetPosition;



            //macierz obrotu os OY na wprost robota
      //      RotationMatrix rmY(rot);
            //Pose currRobotPose=(*currGameState).getRobotPos( robot->getRobotName() );
            //pozycja celu w ukladzie wsp zwiazanych z robotem
        //    Vector2D targetRelPosition=rmY.Inverse()*(goalPose.getPosition()-currRobotPose.getPosition());
            //std::cout<<"targetRelPosition "<<targetRelPosition<<std::endl;
            //obrot w lewo

          //  teta_cel=atan2( (targetRelPosition.y) , (targetRelPosition.x));
           // std::cout<<"teta_cel "<<teta_cel<<std::endl;
          //  if(teta_cel>0 ){
          //      robot->setSpeed(Vector2D(0.0,0.0),-1);
          //  }
            //obrot w prawo
          //  else{
          //      robot->setSpeed(Vector2D(0.0,0.0),1);
          //  }
          //  usleep(10000);



//	    }while(fabs( abs(teta_cel) -M_PI ) > 0.1 );

//	    robot->setSpeed(Vector2D(0.0,0.0),0.0);
	//}
	return false;
}

GoToPose::~GoToPose() {

}
