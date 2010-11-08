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

	double currTime=video.updateGameState(currGameState);
	bool obsPredictionEnable=true;
	/*
	 * za pomoca algorytmu rrt pokieruj robota do celu
	 */
	while(!this->stopTask){
		if(currTime<video.updateGameState(currGameState)){
			currTime=video.updateGameState(currGameState);
			rrt = new RRTPlanner(Config::getInstance().getRRTGoalProb(),
						robot->getRobotName(),obsPredictionEnable,currGameState,goalPose,&path);
			if(rrt->run(currGameState,video.getUpdateDeltaTime()) ){

				GameStatePtr nextState=rrt->getNextState();
				if(nextState.get()==NULL){
					robot->setSpeed(Vector2D(0.0,0.0),0);
					std::cout<<"next state is null"<<std::endl;
					break;
				}
				Pose nextRobotPose=nextState->getRobotPos(robot->getRobotName());

				double rot=(*currGameState).getRobotPos( robot->getRobotName()).get<2>() ;
				//macierz obrotu os OY na wprost robota
				RotationMatrix rmY(rot);

				Pose currRobotPose=(*currGameState).getRobotPos( robot->getRobotName() );

				//pozycja celu w ukladzie wsp zwiazanych z robotem
				Vector2D targetRelPosition=rmY.Inverse()*(nextRobotPose.getPosition()-currRobotPose.getPosition());

				Vector2D robotVel=(*currGameState).getRobotVelocity( robot->getRobotName() );
				Vector2D speed=calculateVelocity( robotVel, Pose(targetRelPosition.x,targetRelPosition.y,0));
				robot->setSpeed(speed,0);
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
