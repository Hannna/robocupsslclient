/*
 * TestRRT.cpp
 *
 *  Created on: 2010-11-03
 *      Author: maciek
 */

#include "TestRRT.h"
#include "../Test/Test.h"
#include "../RRT/RRTPlanner.h"

TestRRT::TestRRT(Robot* robot,Videoserver* video,Pose goalPose) {
	std::cout<<"!!!!!!!!!!!!!!!Start rrt Test"<<std::endl;
	arg1.goalPose=goalPose;
	arg1.video=video;
	arg1.robot=robot;
	this->Arg(reinterpret_cast<void *>(&arg1));

	std::cout<<"!!!!!!!!!!!!!!!Start rrt Test"<<std::endl;


}

void TestRRT::Execute(void *arg){
    std::cout<<"!!!!!!!!!!!!!!!Start rrt Test"<<std::endl;

	threadArgPtr threadArg=reinterpret_cast<threadArgPtr>(arg);

	Pose goalPose=threadArg->goalPose;
	Videoserver* video=threadArg->video;
	Robot* robot=threadArg->robot;

	GameStatePtr currGameState(new GameState());
	int serializedTrees=0;
	std::list<Pose>  path;
	std::ostringstream log;

	double currTime=video->updateGameState(currGameState);
	double prevTime;
	int i=0;
	double sum=0;
	double max=0;
	struct timeval diff;
	struct timeval startTime;

	std::ofstream out(robot->getRobotName().c_str());

	while(true){

		RRTPlanner * rrt;
		while(true){

			if(currTime<video->updateGameState(currGameState)){
				i++;
				prevTime=currTime;
				currTime=video->updateGameState(currGameState);
				sum+=(currTime-prevTime);
				(currTime-prevTime)> max ?  max=(currTime-prevTime): max=max ;

				//measureTime(start,&startTime);
				//diff=measureTime(stop,&startTime);
				//out<<"video diffTime "<<diff.tv_sec<<"[s] "<<diff.tv_usec<<"[us]"<<std::endl;
				//	std::cout<<robot->getRobotName()<<" update delta time "<<video.getUpdateDeltaTime()<<std::endl;

				Pose startPose=(*currGameState).getRobotPos(robot->getRobotName());
				measureTime(start,&startTime);

				//				RRTPlanner rrt (Config::getInstance().getRRTGoalProb(),
				//						robot->getRobotName(),currGameState,goalPose,&path);

				rrt = new RRTPlanner(Config::getInstance().getRRTGoalProb(),
					robot->getRobotName(),currGameState,goalPose,&path);
				if(rrt->run(currGameState,video->getUpdateDeltaTime())){

				/*			diff=measureTime(stop,&startTime);
				out<<"rrt diffTime "<<diff.tv_sec<<"[s] "<<diff.tv_usec<<"[us]"<<std::endl;
				*/
				/*
				std::string fileName("/home/maciek/workspace/magisterka/Debug/");
				fileName.append(robot->getRobotName());
				fileName.append("_rrtTree.xml");
				rrt->serializeTree(fileName.c_str(),serializedTrees++);
				*/
				GameStatePtr nextState=rrt->getNextState();
				if(nextState.get()==NULL){
					std::cout<<"next state is NULL"<<std::endl;
					robot->setSpeed(Vector2D(0.0,0.0),0);
					break;
				}
				Pose nextRobotPose=nextState->getRobotPos(robot->getRobotName());

				log<<"moving robot to nextPose "<<nextRobotPose<<std::endl;
				Logger::getInstance().LogToFile(DBG,log);

				double rot=(*currGameState).getRobotPos( robot->getRobotName()).get<2>() ;
				//macierz obrotu os OY na wprost robota
				RotationMatrix rmY(rot);

				Pose currRobotPose=(*currGameState).getRobotPos( robot->getRobotName() );

				//pozycja celu w ukladzie wsp zwiazanych z robotem
				Vector2D targetRelPosition=rmY.Inverse()*(nextRobotPose.getPosition()-currRobotPose.getPosition());

				Vector2D robotVel=(*currGameState).getRobotVelocity( robot->getRobotName() );
				Vector2D speed=calculateVelocity( robotVel, Pose(targetRelPosition.x,targetRelPosition.y,0));
				robot->setSpeed(speed,0);
				}
				else{
					robot->setSpeed(Vector2D(0.0,0.0),0);
					#ifdef GAZEBO
						SimControl::getInstance().setSimPos(robot->getRobotName().c_str(), goalPose);
					#endif
					break;
				}
			}
	}
/*
	while(!rrt->checkCollisions( (goalPose=rrt->getRandomPose()),0.01) ){
		std::cout<<"getRandomPose"<<std::endl;
	}
*/
	delete rrt;
	}
	std::cout<<robot->getRobotName()<<
	" Mean update delta time=" <<sum/i<<" max update delta time "<<max<<std::endl;
	//	out.close();
}
TestRRT::~TestRRT() {
	// TODO Auto-generated destructor stub
}
