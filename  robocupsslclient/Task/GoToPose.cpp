/*
 * GoToPose.cpp
 *
 *  Created on: 2010-06-07
 *      Author: maciek
 */

#include "GoToPose.h"
#include "../RRT/RRTPlanner.h"

GoToPose::GoToPose(const Pose & pose,Robot * robot):Task(robot),goalPose(pose),serialize( Config::getInstance().isDebugMode() ) {
	this->rrt=NULL;
	currSimTime=0;
	lastSimTime=0;
	currGameState = GameStatePtr( new GameState() );
	currSimTime=video.updateGameState(currGameState);
}

Task* GoToPose::nextTask(){
	return NULL;
}

/*
TaskSharedPtr & GoToPose::nextTask(){
	return TaskSharedPtr(this);
}
*/

Task::status GoToPose::run(void* arg, int steps){

	bool obsPredictionEnable=true;

	//pozycja do ktorej ma dojechac robot w kolejnym kroku
	Pose nextRobotPose;
	//biezaca pozycja robota
	Pose currRobotPose = (*currGameState).getRobotPos( robot->getRobotID() );

	//rotacja robota
	double robotRotation=0;

	LOG_INFO(log, "robot "<<robot->getRobotName()<<" create GoToPose Task, goto "<<this->goalPose <<"from "<< currRobotPose);

	//pozycja celu w ukladzie wsp zwiazanych z robotem
	Vector2D targetRelPosition;
	Vector2D robotCurrentVel;
	Vector2D robotNewVel;
	bool timeMeasure = false;

	static int serializedTrees = 0;

	/*
	 * za pomoca algorytmu rrt pokieruj robota do celu
	 */
	while( !this->stopTask && (steps--)!=0 ){

		if( lastSimTime < ( currSimTime=video.updateGameState(currGameState) ) ){
			lastSimTime = currSimTime;

            currRobotPose=(*currGameState).getRobotPos( robot->getRobotID() );
            RRTPlanner::ErrorCode status;

            if(rrt){
				delete rrt;
				rrt = new RRTPlanner( Config::getInstance().getRRTGoalProb(),
							robot->getRobotName(),obsPredictionEnable,currGameState,goalPose,&path,currSimTime, timeMeasure );
				status=rrt->run( video.getUpdateDeltaTime() );
            }
            else{
            	rrt = new RRTPlanner( Config::getInstance().getRRTGoalProb(),
                    						robot->getRobotName(),obsPredictionEnable,currGameState,goalPose,&path,currSimTime, timeMeasure );
                status=rrt->run( video.getUpdateDeltaTime() );
            }

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
					LOG_DEBUG(log,"From rrtPlanner: next state is null. We arrive target");
					delete rrt;
					rrt = NULL;
					return Task::ok;
				}
				nextRobotPose=nextState->getRobotPos( robot->getRobotID() );

				robotRotation=currRobotPose.get<2>() ;
				//macierz obrotu os OY na wprost robota
				//RotationMatrix rmY(robotRotation);

				//pozycja celu w ukladzie wsp zwiazanych z robotem
				//targetRelPosition=rmY.Inverse()*(nextRobotPose.getPosition()-currRobotPose.getPosition());

				robotCurrentVel=(*currGameState).getRobotVelocity( robot->getRobotID() );

				//robotNewVel=calculateVelocity( robotCurrentVel, Pose(targetRelPosition.x,targetRelPosition.y,0));

				robotNewVel=calculateVelocity( robotCurrentVel, currRobotPose, nextRobotPose);
				double w = robot->calculateAngularVel(*currGameState,robot->getRobotID(), goalPose);
				LOG_TRACE(log,"move robot from"<<currRobotPose<<" to "<<nextRobotPose<<" setVel "<<robotNewVel <<" w"<<w);

				//robot->setRelativeSpeed( robotNewVel, 0);
				robot->setRelativeSpeed( robotNewVel, w );
				//robot->
			}
			else if(status==RRTPlanner::RobotReachGoalPose){
                LOG_DEBUG(log,"From rrtPlanner: RobotReachGoalPose");
                delete rrt;
                rrt = NULL;
                return Task::ok;
			}
			else{
				robot->setRelativeSpeed(Vector2D(0.0,0.0),0);
				LOG_WARN(log,"RRT to "<<goalPose<<" Error: "<<status);
				delete rrt;
				rrt = NULL;
				if(status==RRTPlanner::RobotCollision)
					return Task::collision;

				return Task::error;
			}
		}
	}

	if(this->stopTask)
		return Task::ok;

	return Task::not_completed;
}

GoToPose::~GoToPose() {
	delete this->rrt;
}
