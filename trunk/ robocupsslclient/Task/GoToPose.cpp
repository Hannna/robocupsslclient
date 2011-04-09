/*
 * GoToPose.cpp
 *
 *  Created on: 2010-06-07
 *      Author: maciek
 */

#include "GoToPose.h"
#include "../RRT/RRTPlanner.h"

GoToPose::GoToPose(const Pose & pose,Robot * robot):Task(robot),goalPose(pose) {
	LOG_DEBUG(log, "create GoToPose Task, goto "<<pose);
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

bool GoToPose::run(void* arg, int steps){
	RRTPlanner * rrt=NULL;
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

	//pozycja celu w ukladzie wsp zwiazanych z robotem
	Vector2D targetRelPosition;

	Vector2D robotCurrentVel;
	Vector2D robotNewVel;
	bool timeMeasure = false;

	bool serialize = true;
	static int serializedTrees = 0;

	/*
	 * za pomoca algorytmu rrt pokieruj robota do celu
	 */
	while(!this->stopTask && (steps--)!=0 ){
		if( lastSimTime < ( currSimTime=video.updateGameState(currGameState) ) ){
			lastSimTime=currSimTime;

            currRobotPose=(*currGameState).getRobotPos( robot->getRobotName() );

            if(rrt){
                /*
                if(path.empty()){
                    if( rrt->distanceToNearestObstacle( currRobotPose  ) > 0.1 ){
                        if( nextRobotPose.distance( currRobotPose ) < Config::getInstance().getRRTMinDistance() ){
                            continue;
                        }
                    }
                }
                */
                delete rrt;
            }

			rrt = new RRTPlanner(Config::getInstance().getRRTGoalProb(),
						robot->getRobotName(),obsPredictionEnable,currGameState,goalPose,&path,currSimTime, timeMeasure);

            RRTPlanner::ErrorCode status;
			status=rrt->run( video.getUpdateDeltaTime() );

            if(serialize){
                std::string fileName("/home/maciek/codeblocks/magisterka/bin/Debug/");
                fileName.append(robot->getRobotName());
                fileName.append("_rrtTree.xml");
                rrt->serializeTree(fileName.c_str(),serializedTrees++);
            }

			if( status==RRTPlanner::Success ){

				GameStatePtr nextState=rrt->getNextState();

				if(nextState.get()==NULL){
					robot->setRelativeSpeed(Vector2D(0.0,0.0),0);
					LOG_DEBUG(log,"From rrtPlanner: next state is null.");
					delete rrt;
					return false;
					break;
				}
				nextRobotPose=nextState->getRobotPos(robot->getRobotName());

               //std::cout<<"go to"<<nextRobotPose<<std::endl;

				robotRotation=currRobotPose.get<2>() ;
				//macierz obrotu os OY na wprost robota
				RotationMatrix rmY(robotRotation);

				//pozycja celu w ukladzie wsp zwiazanych z robotem
				targetRelPosition=rmY.Inverse()*(nextRobotPose.getPosition()-currRobotPose.getPosition());

                //std::cout<<"go to"<<targetRelPosition<<std::endl;

				robotCurrentVel=(*currGameState).getRobotVelocity( robot->getRobotName() );

				//std::cout<<"robot->robotCurrentVel"<<robotCurrentVel<<std::endl;

				robotNewVel=calculateVelocity( robotCurrentVel, Pose(targetRelPosition.x,targetRelPosition.y,0));

				//std::cout<<"robot->setSpeed"<<robotNewVel<<std::endl;

				robot->setRelativeSpeed(robotNewVel,
                           calculateAngularVel(*currGameState,robot->getRobotName(), goalPose)
                );

				//delete rrt;
			}
			else if(status==RRTPlanner::RobotReachGoalPose){
                LOG_DEBUG(log,"From rrtPlanner: RobotReachGoalPose");
                delete rrt;
                return true;
			}
			else{
				robot->setRelativeSpeed(Vector2D(0.0,0.0),0);
				LOG_DEBUG(log,"RRT Error: "<<status);
				delete rrt;
				return false;
				break;
			}
		}
	}
	return true;
}

GoToPose::~GoToPose() {

}
