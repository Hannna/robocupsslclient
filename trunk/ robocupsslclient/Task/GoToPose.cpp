/*
 * GoToPose.cpp
 *
 *  Created on: 2010-06-07
 *      Author: maciek
 */

#include "GoToPose.h"
#include "../RRT/RRTPlanner.h"

GoToPose::GoToPose(const Pose & pose,Robot * robot):Task(robot),goalPose(pose) {
	std::cout<<"create GoToPose Task, goto "<<pose<<std::endl;
}

bool GoToPose::execute(){
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

	//pozycja celu w ukladzie wsp zwiazanych z robotem
	Vector2D targetRelPosition;

	Vector2D robotCurrentVel;
	Vector2D robotNewVel;
	/*
	 * za pomoca algorytmu rrt pokieruj robota do celu
	 */
	while(!this->stopTask){
		if( lastSimTime < ( currSimTime=video.updateGameState(currGameState) ) ){
			lastSimTime=currSimTime;

			rrt = new RRTPlanner(Config::getInstance().getRRTGoalProb(),
						robot->getRobotName(),obsPredictionEnable,currGameState,goalPose,&path);

			if(rrt->run(currGameState,video.getUpdateDeltaTime()) ){

				GameStatePtr nextState=rrt->getNextState();
				if(nextState.get()==NULL){
					robot->setSpeed(Vector2D(0.0,0.0),0);
					std::cout<<"next state is null"<<std::endl;
					break;
				}
				nextRobotPose=nextState->getRobotPos(robot->getRobotName());

				robotRotation=(*currGameState).getRobotPos( robot->getRobotName()).get<2>() ;
				//macierz obrotu os OY na wprost robota
				RotationMatrix rmY(robotRotation);

				currRobotPose=(*currGameState).getRobotPos( robot->getRobotName() );

				//pozycja celu w ukladzie wsp zwiazanych z robotem
				targetRelPosition=rmY.Inverse()*(nextRobotPose.getPosition()-currRobotPose.getPosition());

				robotCurrentVel=(*currGameState).getRobotVelocity( robot->getRobotName() );
				robotNewVel=calculateVelocity( robotCurrentVel, Pose(targetRelPosition.x,targetRelPosition.y,0));
				robot->setSpeed(robotNewVel,0);

				delete rrt;
			}
			else{
				robot->setSpeed(Vector2D(0.0,0.0),0);
				std::cout<<"reache goal state"<<std::endl;
				break;
			}
		}
	}
	/*
	 * jesli robot jest u celu to wykonaj korekte rotacji
	 */
	if(!this->stopTask){
		double rot=(*currGameState).getRobotPos( robot->getRobotName()).get<2>() ;
		//macierz obrotu os OY na wprost robota
		RotationMatrix rmY(rot);
		Pose currRobotPose=(*currGameState).getRobotPos( robot->getRobotName() );
		//pozycja celu w ukladzie wsp zwiazanych z robotem
		Vector2D targetRelPosition=rmY.Inverse()*(goalPose.getPosition()-currRobotPose.getPosition());
		std::cout<<"targetRelPosition "<<targetRelPosition<<std::endl;


	}
	return true;
}

GoToPose::~GoToPose() {

}
