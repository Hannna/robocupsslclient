/*
 * test.cpp
 *
 *  Created on: 2009-12-30
 *      Author: maciek
 */

#include "Test.h"

#include "../RRT/RRTPlanner.h"
#include "../RotationMatrix/RotationMatrix.h"
#include "../Task/GoToPose.h"

const std::string ifaceName="::position_iface";

void testPose(Robot& robot,Pose newPose){
#ifdef GAZEBO
	Pose pose;
	SimControl::getInstance().getModelPos(robot.getRobotName(),pose);
	std::cout<<"\nstart test with position x= "<<pose.get<0>()<<" y= "<<pose.get<1>()<<" yaw= "<<pose.get<2>()<<std::endl;
	sleep(2);
	std::cout<<"try to set robot position x= "<<newPose.get<0>()<<" y= "<<newPose.get<1>()<<" yaw= "<<newPose.get<2>()<<std::endl;
	SimControl::getInstance().setSimPos(robot.getRobotName().c_str(),newPose);
	SimControl::getInstance().getModelPos(robot.getRobotName(),pose);
	std::cout<<"ending with position x= "<<pose.get<0>()<<" y= "<<pose.get<1>()<<" yaw= "<<pose.get<2>()<<std::endl;
#endif
}
void testRotation(Videoserver & video,Robot& robot){
	GameStatePtr gameState(new GameState());
	double deltaTime=0;

	std::vector<Vector2D> goalPositions;
	goalPositions.push_back(Vector2D(1,0));
	goalPositions.push_back(Vector2D(-1,0));
	goalPositions.push_back(Vector2D(0,1));
	goalPositions.push_back(Vector2D(0,-1));
	video.updateGameState(gameState);

	std::vector<Vector2D>::iterator ii=goalPositions.begin();
	for(;ii!=goalPositions.end();ii++){
		Vector2D goToPosition=*ii;
		std::cout<<"go to position "<<goToPosition<<std::endl;
		(*gameState).updateRobotVel(robot.getRobotName(),robot.getVelocity() );
		deltaTime=video.getUpdateDeltaTime();

		double rot=(*gameState).getRobotPos( robot.getRobotName()).get<2>() ;
		std::cout<<"rotacja robota "<<rot<<std::endl;
		//macierz obrotu os OY na wprost robota
		RotationMatrix rmY(rot);

		Pose currRobotPose=(*gameState).getRobotPos( robot.getRobotName() );
		//pozycja robota w ukladzie wsp zw z plansza
		Vector2D currentPosition( currRobotPose.get<0>(),currRobotPose.get<1>() );

		//pozycja celu w ukladzie wsp zwiazanych z robotem
		Vector2D targetPosition=rmY.Inverse()*(goToPosition-currentPosition);

		//rotacja do celu
		double teta_cel=atan2( (targetPosition.y) , (targetPosition.x));
		//macierz obrotu cel na wprost robota
		//RotationMatrix rm(teta_cel);
		//targetPosition = rmY.Inverse() * targetPosition;

		std::cout<<"rotacja do celu"<<teta_cel<<"pozycja celu w ukl wsp zw z robotem "<<targetPosition<<std::endl;

	}
}
void testMotion(Pose goalPose,Videoserver & video,Robot& robot){
	GameStatePtr gameState(new GameState());
	double deltaTime=0;
	Pose currRobotPose;
	video.updateGameState(gameState);
	Vector2D speed;
	video.updateGameState(gameState);

	do {
		video.updateGameState(gameState);
		(*gameState).updateRobotVel(robot.getRobotName(),robot.getVelocity() );
		deltaTime=video.getUpdateDeltaTime();

		double rot=(*gameState).getRobotPos( robot.getRobotName()).get<2>() ;
		std::cout<<"#####################################"<<speed<<std::endl;
		//macierz obrotu os OY na wprost robota
		RotationMatrix rmY(rot);

		currRobotPose=(*gameState).getRobotPos( robot.getRobotName() );
		//pozycja robota w ukladzie wsp zw z plansza
		Vector2D currentAbsPosition( currRobotPose.get<0>(),currRobotPose.get<1>() );
		Vector2D goToAbsPosition(goalPose.get<0>(),goalPose.get<1>());

		//pozycja celu w ukladzie wsp zwiazanych z robotem
		Vector2D targetRelPosition=rmY.Inverse()*(goToAbsPosition-currentAbsPosition);

		Vector2D robotVel=(*gameState).getRobotVelocity( robot.getRobotName() );
		speed=calculateVelocity( robotVel, Pose(targetRelPosition.x,targetRelPosition.y,0));

		std::cout<<"set speed "<<speed<<std::endl;
		robot.setSpeed(speed,0);
	}while( sqrt( pow( ( goalPose.get<0>()-currRobotPose.get<0>() ),2) + pow( ( goalPose.get<1>() - currRobotPose.get<1>() ),2) ) > 0.001 );

}
void testVel(Vector2D speed,double yaw,Robot& robot,time_t testTime){
#ifdef GAZEBO
	time_t startTime=time(NULL);
	speed=speed*Config::getInstance().getSpeedFactor();
	std::cout<<"start test with vx="<<speed.x<<" vy="<<speed.y<<" w="<<yaw<<std::endl;
	Pose currPosition,prevPosition;
	SimControl::getInstance().getModelPos(robot.getRobotName(),prevPosition);
	double currSimTime,prevSimTime;
	double vx,vy;
	prevSimTime=SimControl::getInstance().getSimTime();
	while(time(NULL)-startTime<testTime){
		//currSimTime=SimControl::getInstance().getSimTime();
		robot.setSpeed(speed,yaw);
		//Pose position;
		//std::map<std::string,Pose > positions;
		SimControl::getInstance().getModelPos(robot.getRobotName(),currPosition);
		currSimTime=SimControl::getInstance().getSimTime();

		//vx=(currPosition.get<0>()-prevPosition.get<0>())/(currSimTime-prevSimTime);
		//vy=(currPosition.get<1>()-prevPosition.get<1>())/(currSimTime-prevSimTime);
		//Logger::getInstance().LogToFile(DBG,"vx=%f vy=%f simTime=%f",vx,vy,currSimTime);

		//double rot=currPosition.get<2>();
		//macierz obrotu os OY na wprost robota
		//RotationMatrix rmY(rot);
		Vector2D tmp1(robot.getVelocity().first);

		Vector2D tmp2(vx,vy);
		//Vector2D tmp2=rmY.Inverse()*tmp1;
		///Vector2D targetPosition=rmX.Inverse()*(goToPosition-currentPosition);
		//std::cout<<"rot from gazebo"<<rot<<std::endl;

		std::cout<<"calculated robotVelocity "<<tmp2<<std::endl;
		std::cout<<"robotVelocity "<<tmp1<<std::endl;
		prevSimTime=currSimTime;
		prevPosition=currPosition;

		//SimControl::getInstance().getAllPos(positions);
		usleep(100000);//100ms
	}

	robot.setSpeed(Vector2D(0.0,0.0),0);
	getchar();
	SimControl::getInstance().restart();
	//usleep(1000000);
#endif
	return;
}

void checkAcceleration(Vector2D speed,Videoserver & video,Robot& robot){
	GameStatePtr gameState(new GameState());
	double deltaTime=0;

	video.updateGameState(gameState);
	bool exit=false;
	Vector2D maxSpeed;
	while(!exit){
		video.updateGameState(gameState);
		deltaTime+=video.getUpdateDeltaTime();
		robot.setSpeed(speed,0);
		(*gameState).updateRobotVel(robot.getRobotName(),robot.getVelocity() );

		if( ( (fabs( (*gameState).getRobotVelocity( robot.getRobotName() ).length() - speed.length() ) <0.01 ) && !exit )
				|| ( (fabs( (*gameState).getRobotVelocity( robot.getRobotName() ).length() ) <0.01 )  && exit ) ){
			if(!exit){
				std::cout<<"velocity "<<(*gameState).getRobotVelocity( robot.getRobotName())<<" delta Time "<<deltaTime<<" acc="
					<<(*gameState).getRobotVelocity( robot.getRobotName()).length()/deltaTime<<std::endl;
				maxSpeed=(*gameState).getRobotVelocity( robot.getRobotName());
			}
			else{
				std::cout<<"velocity "<<(*gameState).getRobotVelocity( robot.getRobotName())<<" delta Time "<<deltaTime<<" dcc="
							<<maxSpeed.length()/deltaTime<<std::endl;
			}
			speed=Vector2D(0,0);
			deltaTime=0;
			if(exit)break;
			exit=true;
		}
	}
	robot.setSpeed(Vector2D(0,0),0);

}
void testTaskThread(){
	Robot redRobot0(std::string("red0"),ifaceName);
	Robot redRobot1(std::string("red1"),ifaceName);
	Robot redRobot2(std::string("red2"),ifaceName);

	Robot blueRobot0(std::string("blue0"),ifaceName);
	Robot blueRobot1(std::string("blue1"),ifaceName);
	Robot blueRobot2(std::string("blue2"),ifaceName);

	pthread_t red0,red1,red2;
	pthread_t blue0,blue1,blue2;
	pthread_t video_t;
	pthread_attr_t  attr;
	pthread_attr_init(&attr);

	//uruchom videoserver
	Videoserver::getInstance().Start(NULL);

    while(true){
        pthread_create(&red0, &attr, testTask, (void *) &redRobot0);
        pthread_join(red0,NULL);
		#ifdef GAZEBO
			SimControl::getInstance().restart();
		#endif
    }


	/*
	pthread_create(&red1, &attr, RRTThread, (void *) &arg2);
	pthread_join(red1,NULL);

	pthread_create(&red2, &attr, RRTThread, (void *) &arg3);
	pthread_join(red2,NULL);

	pthread_create(&blue0, &attr, RRTThread, (void *) &arg4);
	pthread_join(blue0,NULL);

	pthread_create(&blue1, &attr, RRTThread, (void *) &arg5);
	pthread_join(blue1,NULL);

	pthread_create(&blue2, &attr, RRTThread, (void *) &arg6);
	pthread_join(blue2,NULL);
*/
}

void * testTask(void * arg){
    std::cout<<"ACHTUNG start test task"<<std::endl;
    std::cout<<"ACHTUNG start test task"<<std::endl;
    std::cout<<"ACHTUNG start test task"<<std::endl;

	Robot * robot=reinterpret_cast<Robot *>(arg);
	GameStatePtr gameState(new GameState());
	Videoserver::getInstance().updateGameState(gameState);
	//Pose pose(5.0,1.0,0);
	GoToPose goToPose( (*gameState).getBallPos(),robot);
	goToPose.execute();
	std::cout<<"exit from task"<<std::endl;
	return 0;
}

void testRRTThread(Videoserver & video){
	Robot redRobot0(std::string("red0"),ifaceName);
	/*
	Robot redRobot1(std::string("red1"),ifaceName);
	Robot redRobot2(std::string("red2"),ifaceName);

	Robot blueRobot0(std::string("blue0"),ifaceName);
	Robot blueRobot1(std::string("blue1"),ifaceName);
	Robot blueRobot2(std::string("blue2"),ifaceName);
*/
	pthread_t red0,red1,red2;
	pthread_t blue0,blue1,blue2;
	pthread_t video_t;
	pthread_attr_t  attr;
	pthread_attr_init(&attr);

	struct threadArg arg1;
	arg1.goalPose=Pose(5.5,2.5,0);
	arg1.video=&video;
	arg1.robot=&redRobot0;

/*
	struct threadArg arg2;
	arg2.goalPose=Pose(1.5,2.5,0);
	arg2.video=&video;
	arg2.robot=&redRobot1;

	struct threadArg arg3;
	arg3.goalPose=Pose(5.3,0.5,0);
	arg3.video=&video;
	arg3.robot=&redRobot2;

	struct threadArg arg4;
	arg4.goalPose=Pose(1.3,0.5,0);
	arg4.video=&video;
	arg4.robot=&blueRobot0;

	struct threadArg arg5;
	arg5.goalPose=Pose(2.3,1.5,0);
	arg5.video=&video;
	arg5.robot=&blueRobot1;

	struct threadArg arg6;
	arg6.goalPose=Pose(2.6,0.5,0);
	arg6.video=&video;
	arg6.robot=&blueRobot2;
*/

//	pthread_create(&video_t, &attr, runVideoserver, NULL);

//	pthread_create(&red0, &attr, RRTThread, (void *) &arg1);
//	pthread_join(red0,NULL);


	/*
	pthread_create(&red1, &attr, RRTThread, (void *) &arg2);
	pthread_join(red1,NULL);

	pthread_create(&red2, &attr, RRTThread, (void *) &arg3);
	pthread_join(red2,NULL);

	pthread_create(&blue0, &attr, RRTThread, (void *) &arg4);
	pthread_join(blue0,NULL);

	pthread_create(&blue1, &attr, RRTThread, (void *) &arg5);
	pthread_join(blue1,NULL);

	pthread_create(&blue2, &attr, RRTThread, (void *) &arg6);
	pthread_join(blue2,NULL);
*/
}
void* RRTThread(void * arg){
	threadArgPtr threadArg=reinterpret_cast<threadArgPtr>(arg);
	testRRT(threadArg->goalPose,*threadArg->video,threadArg->robot);
	return 0;
}
void testRRT(Pose goalPose,Videoserver & video,Robot* robot){
	GameStatePtr currGameState(new GameState());
	int serializedTrees=0;
	std::list<Pose>  path;
	std::ostringstream log;

	double currTime=video.updateGameState(currGameState);
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

			if(currTime<video.updateGameState(currGameState)){
				i++;

				prevTime=currTime;
				currTime=video.updateGameState(currGameState);
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
				if(rrt->run(currGameState,video.getUpdateDeltaTime())){

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

		while(!rrt->checkCollisions( (goalPose=rrt->getRandomPose()),0.01) ){
			std::cout<<"getRandomPose"<<std::endl;
		}
		delete rrt;
	}
	std::cout<<robot->getRobotName()<<
			" Mean update delta time=" <<sum/i<<" max update delta time "<<max<<std::endl;
//	out.close();
}
