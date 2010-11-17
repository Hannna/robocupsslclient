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
	double vx=0,vy=0;
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

void checkAcceleration(Vector2D speed,Robot& robot){
	std::cout<<"checkAcceleration tests"<<std::endl;

	GameStatePtr gameState(new GameState());
	double deltaTime=0;
	double currSimTime=0 ,prevSimTime=Videoserver::getInstance().updateGameState(gameState);

	bool exit=false;
	bool brake=false;
	Vector2D maxSpeed;

	robot.setSpeed(speed,0);
	while(true){
	    currSimTime=Videoserver::getInstance().updateGameState(gameState);

        if( prevSimTime <  currSimTime ){
            std::cout<<"currSimTime"<<currSimTime<<std::endl;
            deltaTime+=(currSimTime - prevSimTime);
            prevSimTime=currSimTime;

            //robot.setSpeed(speed,0);

            //(*gameState).updateRobotVel(robot.getRobotName(),robot.getVelocity() );


            if( ( (fabs( (*gameState).getRobotVelocity( robot.getRobotName() ).length() - speed.length() ) <0.01 ) && !brake )
                    || ( (fabs( (*gameState).getRobotVelocity( robot.getRobotName() ).length() ) <0.01 )  && brake ) )
            {
                if(!brake){
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
                robot.setSpeed(speed,0);
                if(exit)
                    break;
                exit=true;
                brake=true;
            }
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
	//pthread_t video_t;
	pthread_attr_t  attr;
	pthread_attr_init(&attr);

	//uruchom videoserver
	Videoserver::getInstance().start(NULL);

    while(true){
        pthread_create(&red0, &attr, testTask, (void *) &redRobot0);


        //pthread_create(&red1, &attr, testTask, (void *) &redRobot1);
        //pthread_create(&red2, &attr, testTask, (void *) &redRobot2);
        //pthread_create(&blue0, &attr,testTask, (void *) &blueRobot0);
       //pthread_create(&blue1, &attr, testTask, (void *) &blueRobot1);
       // pthread_create(&blue2, &attr, testTask, (void *) &blueRobot2);

        pthread_join(red0,NULL);
        //pthread_join(red1,NULL);
        //pthread_join(red2,NULL);
        //pthread_join(blue0,NULL);
        //pthread_join(blue1,NULL);
        //pthread_join(blue2,NULL);

        #ifdef GAZEBO
			SimControl::getInstance().restart();
			//TODO: poczekaj az swiat sie zrestartuje!!!!!
		#endif

   }
}

void * testTask(void * arg){
    //std::cout<<"start testTask"<<std::endl;

	Robot * robot=reinterpret_cast<Robot *>(arg);
	GameStatePtr gameState(new GameState());
	Videoserver::getInstance().updateGameState(gameState);

	GoToPose goToPose( (*gameState).getBallPos(),robot);
	//GoToPose goToPose( Pose(6.0,4.0,0.0),robot);
	goToPose.execute();
	//std::cout<<"exit from task"<<std::endl;
	return 0;
}

void testSingleRRTThread(){
    std::cout<<"start testSingleRRTThread"<<std::endl;
	Robot redRobot0(std::string("red0"),ifaceName);
	Robot redRobot1(std::string("red1"),ifaceName);
	Robot redRobot2(std::string("red2"),ifaceName);

	Robot blueRobot0(std::string("blue0"),ifaceName);
	Robot blueRobot1(std::string("blue1"),ifaceName);
	Robot blueRobot2(std::string("blue2"),ifaceName);

    Videoserver::getInstance().start(NULL);

	TestRRT testRRT(&redRobot0,Pose(4.0,6.0,0));
    testRRT.join();

    std::cout<<"exit from testSingleRRTThread"<<std::endl;

}
void testMultiRRTThread(){
	Robot redRobot0(std::string("red0"),ifaceName);
	Robot redRobot1(std::string("red1"),ifaceName);
	Robot redRobot2(std::string("red2"),ifaceName);

	Robot blueRobot0(std::string("blue0"),ifaceName);
	Robot blueRobot1(std::string("blue1"),ifaceName);
	Robot blueRobot2(std::string("blue2"),ifaceName);


    Videoserver::getInstance().start(NULL);


	TestRRT testRRTred0(&redRobot0,Pose(5.5,2.5,0));
	//testRRTred0.start(NULL);
	/*
	TestRRT testRRTred1(&redRobot1,Pose(1.5,2.5,0));
	testRRTred1.start(NULL);
	TestRRT testRRTred2(&redRobot2,Pose(5.3,0.5,0));
	testRRTred2.start(NULL);

	TestRRT testRRTblue0(&blueRobot0,Pose(1.3,0.5,0));
	testRRTblue0.start(NULL);
	TestRRT testRRTblue1(&blueRobot1,Pose(2.3,1.5,0));
	testRRTblue1.start(NULL);
	TestRRT testRRTblue2(&blueRobot2,Pose(2.6,0.5,0));
	testRRTblue2.start(NULL);
*/
	testRRTred0.join();
/*	testRRTred1.join();
	testRRTred2.join();

	testRRTblue0.join();
	testRRTblue1.join();
	testRRTblue2.join();
*/
	std::cout<<"exit from testMultiRRTThread"<<std::endl;
}
