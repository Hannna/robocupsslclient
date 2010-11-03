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
#include "../TestRRT/TestRRT.h"

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

	pthread_t red0;//,red1,red2;
	//pthread_t blue0,blue1,blue2;
	//pthread_t video_t;
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

	Robot * robot=reinterpret_cast<Robot *>(arg);
	GameStatePtr gameState(new GameState());
	Videoserver::getInstance().updateGameState(gameState);
	GoToPose goToPose( (*gameState).getBallPos(),robot);
	goToPose.execute();
	std::cout<<"exit from task"<<std::endl;
	return 0;
}

void testSingleRRTThread(Videoserver & video){
	Robot redRobot0(std::string("red0"),ifaceName);
	TestRRT testRRT(&redRobot0,&video,Pose(5.5,2.5,0));
	testRRT.Start(NULL);
}
void testMultiRRTThread(Videoserver & video){
	Robot redRobot0(std::string("red0"),ifaceName);
	Robot redRobot1(std::string("red1"),ifaceName);
	Robot redRobot2(std::string("red2"),ifaceName);

	Robot blueRobot0(std::string("blue0"),ifaceName);
	Robot blueRobot1(std::string("blue1"),ifaceName);
	Robot blueRobot2(std::string("blue2"),ifaceName);


	TestRRT testRRTred0(&redRobot0,&video,Pose(5.5,2.5,0));
	testRRTred0.Start(NULL);
	TestRRT testRRTred1(&redRobot1,&video,Pose(1.5,2.5,0));
	testRRTred1.Start(NULL);
	TestRRT testRRTred2(&redRobot2,&video,Pose(5.3,0.5,0));
	testRRTred2.Start(NULL);

	TestRRT testRRTblue0(&blueRobot0,&video,Pose(1.3,0.5,0));
	testRRTblue0.Start(NULL);
	TestRRT testRRTblue1(&blueRobot1,&video,Pose(2.3,1.5,0));
	testRRTblue1.Start(NULL);
	TestRRT testRRTblue2(&blueRobot2,&video,Pose(2.6,0.5,0));
	testRRTblue2.Start(NULL);
/*
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
}
