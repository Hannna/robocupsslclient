/*
 * TestRRT.cpp
 *
 *  Created on: 2010-11-03
 *      Author: maciek
 */

#include "TestRRT.h"
#include "../Test/Test.h"
#include "../RRT/RRTPlanner.h"
#include "../Task/GoToPose.h"

TestRRT::TestRRT(Robot* robot,Pose goalPose,bool serialize_ ): serialize(serialize_) {
	arg1.goalPose=goalPose;
	arg1.video=NULL;
	arg1.robot=robot;

	this->start(reinterpret_cast<void *>(&this->arg1));

}

void TestRRT::execute(void *args){
    std::cout<<"Start rrt test"<<std::endl;
	threadArgPtr threadArg=reinterpret_cast<struct threadArg *>(args);
    Pose goalPose;//=threadArg->goalPose;
    Videoserver* video;//=&Videoserver::getInstance();//threadArg->video;
    Robot* robot;//=threadArg->robot;
    if(threadArg!=NULL){
        goalPose=threadArg->goalPose;
        video=&Videoserver::getInstance();//threadArg->video;
        robot=threadArg->robot;
    }
    else
        return;

	GameStatePtr currGameState(new GameState());
//	int serializedTrees=0;
	std::list<Pose>  path;
	std::ostringstream log;

	//biezacy czas symulacji
	double currSimTime;
#ifdef GAZEBO
	do{
	    currSimTime=video->updateGameState(currGameState);
	}
	while(currSimTime<1);
#endif
  //  std::cout<<"Create single rrt test for robot"<<robot->getRobotName()<<" go from "<<(*currGameState)<<" to pose "<<goalPose<<std::endl;

	//ostatni czas pobrany z symulacji
	double prevSimTime;
	//czas rozpoczecia algorytmu
	const double startSimTime=currSimTime;
	//maksymalny odstep pomiedzy wystawieniem nowych sterowań
	double max=0;
	//numer kolejnej iteracji algorytmu
	int step=0;
	//struct timeval diff;
	//struct timeval startTime;

	std::ofstream out(robot->getRobotName().c_str());

//	GoToPose goToPose( goalPose,robot);
//	goToPose.execute();



    int serializedTrees=0;
	while(true){
	    #ifdef DEBUG
            std::cout<<"Try to plan path to "<<goalPose<<" from "<<currGameState->getRobotPos(robot->getRobotName())<<" for robot "<<robot->getRobotName()<<std::endl;
		#endif
		RRTPlanner  *rrt;

        //biezaca rotacja sterowanego robota
		double robotRotation=0;
		//biezaca predkosc robota
		Vector2D currRobotVel;
		//nowe wyznaczone sterowanie
		Vector2D newRobotVel;
		//pozcja do celu w ukladzie wsp zw z robotem
		Vector2D targetRelPosition;
        //pozycja poczatkowa robota
        Pose startPose;
        //pozycja docelowa w kolejnym kroku algorytmu
        Pose nextRobotPose;
        //czy nalezy przewidywac ruch przeszkod
        bool obsPredictionEnable=true;

        max=0;
        RRTPlanner::ErrorCode rrtStatus;

		while(true){
			//jesli videoserwer wykonal aktualizacje polozen robotow
			if( currSimTime < video->updateGameState(currGameState) )
			{	step++;
                prevSimTime=currSimTime;
                currSimTime=currGameState->getSimTime();

				//currSimTime=video->updateGameState(currGameState);

				(currSimTime-prevSimTime) > max ?  max=(currSimTime-prevSimTime): max=max ;

				//measureTime(start,&startTime);
				//diff=measureTime(stop,&startTime);
				//out<<"video diffTime "<<diff.tv_sec<<"[s] "<<diff.tv_usec<<"[us]"<<std::endl;
				//	std::cout<<robot->getRobotName()<<" update delta time "<<video.getUpdateDeltaTime()<<std::endl;

				startPose=(*currGameState).getRobotPos(robot->getRobotName());
				//measureTime(start,&startTime);

				rrt = new RRTPlanner(
						Config::getInstance().getRRTGoalProb(),
						robot->getRobotName(),
						obsPredictionEnable,
						currGameState,
						goalPose,
						&path,
						currSimTime
					);
				if( (rrtStatus=rrt->run(video->getUpdateDeltaTime() ) )== RRTPlanner::Success ){

					//pomiar czestotliowsci
					/*	diff=measureTime(stop,&startTime);
						out<<"rrt diffTime "<<diff.tv_sec<<"[s] "<<diff.tv_usec<<"[us]"<<std::endl;
					*/

					//zapis drzewa rrt do pliku
					if(serialize){
						std::string fileName("/home/maciek/codeblocks/magisterka/bin/Debug/");
						fileName.append(robot->getRobotName());
						fileName.append("_rrtTree.xml");
						rrt->serializeTree(fileName.c_str(),serializedTrees++);
					}

					GameStatePtr nextState=rrt->getNextState();
					if(nextState.get()==NULL){
						std::cout<<"rrt planner return next state as NULL"<<std::endl;
						robot->setRelativeSpeed(Vector2D(0.0,0.0),0);
						break;
					}

					nextRobotPose=nextState->getRobotPos(robot->getRobotName());

                    #ifdef DEBUG
                        std::cout<<"moving robot to nextPose "<<nextRobotPose<<" from "<<startPose<<std::endl;
                        Logger::getInstance().LogToFile(DBG,log);
                    #endif

					//biezaca rotacja robota
					robotRotation=(*currGameState).getRobotPos( robot->getRobotName()).get<2>() ;

					//macierz obrotu os OY na wprost robota
					RotationMatrix rmY(robotRotation);

					//pozycja celu w ukladzie wsp zwiazanych z robotem
					targetRelPosition=rmY.Inverse()*(nextRobotPose.getPosition() - startPose.getPosition());
                    #ifdef DEBUG
                        std::cout<<"targetRelPosition "<<targetRelPosition<<std::endl;
                    #endif
					//pobiez biezaca predkosc robota
					currRobotVel=(*currGameState).getRobotVelocity( robot->getRobotName() );
					#ifdef DEBUG
                        std::cout<<"currRobotVel "<<currRobotVel<<std::endl;
                    #endif
					//oblicz nowe sterowanie
					newRobotVel=calculateVelocity( currRobotVel, Pose(targetRelPosition.x,targetRelPosition.y,0));
					//ustaw sterowanie
					#ifdef DEBUG
                        std::cout<<"set speed "<<newRobotVel<<std::endl;
                    #endif
					robot->setRelativeSpeed(newRobotVel,0);
					delete rrt;
				}
				else if(rrtStatus==RRTPlanner::RobotReachGoalPose){
				    //std::cout<<"robot u celu"<<std::endl;
					robot->setRelativeSpeed(Vector2D(0.0,0.0),0);
					sleep(2);
					delete rrt;
					break;
				}
				else{

					robot->setRelativeSpeed(Vector2D(0.0,0.0),0);
					sleep(2);
					#ifdef GAZEBO
					//SimControl::getInstance().pause();
					//SimControl::getInstance().restart();
					//SimControl::getInstance().resume();
					sleep(2);
						//SimControl::getInstance().setSimPos(robot->getRobotName().c_str(), goalPose);
					#endif

					if(rrtStatus==RRTPlanner::BadTarget){
					    std::cout<<"RRTPlanner::BadTarget"<<std::endl;

					}
					else if(rrtStatus==RRTPlanner::TargetInsideObstacle){
                        std::cout<<"RRTPlanner::TargetInsideObstacle"<<std::endl;
					}
					else if(rrtStatus==RRTPlanner::RobotCollision){
                        std::cout<<robot->getRobotName()<<" RRTPlanner::RobotCollision"<<std::endl;
                        return;
					}
					delete rrt;
					break;
				}

			}
		}

        #ifdef DEBUG
            std::cout<<robot->getRobotName()<<
            " Mean update delta time=" <<(currSimTime-startSimTime)/step<<" max update delta time "<<max<<std::endl;
		#endif
		//wylosuj kolejny pkt docelowy
        goalPose=RRTPlanner::getRandomPose();
	}

	//	out.close();

}
void TestRRT::joinThread(){
    this->join();
}
TestRRT::~TestRRT() {
	// TODO Auto-generated destructor stub
}
