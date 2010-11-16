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

TestRRT::TestRRT(Robot* robot,Pose goalPose) {
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



	while(true){
		std::cout<<"Try to plan path to "<<goalPose<<" from "<<currGameState->getRobotPos(robot->getRobotName())<<" for robot "<<robot->getRobotName()<<std::endl;
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
        bool obsPredictionEnable=false;

        max=0;

		while(true){
			//jesli videoserwer wykonal aktualizacje polozen robotow
			if( currSimTime < video->updateGameState(currGameState) )
			{	step++;
				prevSimTime=currSimTime;
				currSimTime=video->updateGameState(currGameState);

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
						&path
					);
				if( rrt->run(video->getUpdateDeltaTime() ) ){

                    //currRobotVel=(*currGameState).getRobotVelocity( robot->getRobotName() );
                    //std::cout<<"currRobotVel!!!!!!!!! "<<currRobotVel<<std::endl;


					//pomiar czestotliowsci
					/*	diff=measureTime(stop,&startTime);
						out<<"rrt diffTime "<<diff.tv_sec<<"[s] "<<diff.tv_usec<<"[us]"<<std::endl;
					*/

					//zapis drzewa rrt do pliku
					/*
						std::string fileName("/home/maciek/workspace/magisterka/Debug/");
						fileName.append(robot->getRobotName());
						fileName.append("_rrtTree.xml");
						rrt->serializeTree(fileName.c_str(),serializedTrees++);
					*/

					GameStatePtr nextState=rrt->getNextState();
					if(nextState.get()==NULL){
						std::cout<<"rrt planner return next state as NULL"<<std::endl;
						robot->setSpeed(Vector2D(0.0,0.0),0);
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
					robot->setSpeed(newRobotVel,0);
				}
				else{
				    std::cout<<"run retrun false"<<std::endl;
					robot->setSpeed(Vector2D(0.0,0.0),0);
					#ifdef GAZEBO
						SimControl::getInstance().setSimPos(robot->getRobotName().c_str(), goalPose);
					#endif
					delete rrt;
					break;
				}

				delete rrt;
			}
		}

        std::cout<<robot->getRobotName()<<
        " Mean update delta time=" <<(currSimTime-startSimTime)/step<<" max update delta time "<<max<<std::endl;
		//wylosuj kolejny pkt docelowy
        goalPose=RRTPlanner::getRandomPose();
	}

	//	out.close();

}
TestRRT::~TestRRT() {
	// TODO Auto-generated destructor stub
}
