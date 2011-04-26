/*
 * test.cpp
 *
 *  Created on: 2009-12-30
 *      Author: maciek
 */

#include "Test.h"

#include "../RRT/RRTPlanner.h"
#include "../RotationMatrix/RotationMatrix.h"
#include "../SimControl/SimControl.h"

#include "../EvaluationModule/EvaluationModule.h"

#include "../Task/GoToPose.h"
#include "../Task/Task.h"
#include "../Task/KickBall.h"

#include "../TestRRT/TestRRT.h"
#include "../AbstractTactic/AbstractTactic.h"
#include "../Tactics/ShootTactic.h"



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
		(*gameState).updateRobotVel(robot.getRobotID(),robot.getVelocity() );
		deltaTime=video.getUpdateDeltaTime();

		double rot=(*gameState).getRobotPos( robot.getRobotID() ).get<2>() ;
		std::cout<<"rotacja robota "<<rot<<std::endl;
		//macierz obrotu os OY na wprost robota
		RotationMatrix rmY(rot);

		Pose currRobotPose=(*gameState).getRobotPos( robot.getRobotID() );
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
		(*gameState).updateRobotVel(robot.getRobotID() ,robot.getVelocity() );
		deltaTime=video.getUpdateDeltaTime();

		double rot=(*gameState).getRobotPos( robot.getRobotID() ).get<2>() ;
		std::cout<<"#####################################"<<speed<<std::endl;
		//macierz obrotu os OY na wprost robota
		//RotationMatrix rmY(rot);

		currRobotPose=(*gameState).getRobotPos( robot.getRobotID() );
		//pozycja robota w ukladzie wsp zw z plansza
		Vector2D currentAbsPosition( currRobotPose.get<0>(),currRobotPose.get<1>() );
		Vector2D goToAbsPosition(goalPose.get<0>(),goalPose.get<1>());

		//pozycja celu w ukladzie wsp zwiazanych z robotem
		//Vector2D targetRelPosition=rmY.Inverse()*(goToAbsPosition-currentAbsPosition);

		Vector2D robotVel=(*gameState).getRobotVelocity( robot.getRobotID() );
		//speed=calculateVelocity( robotVel, Pose(targetRelPosition.x,targetRelPosition.y,0));
		speed=calculateVelocity( robotVel,currRobotPose, goalPose);

		std::cout<<"set speed "<<speed<<std::endl;
		robot.setRelativeSpeed(speed,0);
	}while( sqrt( pow( ( goalPose.get<0>()-currRobotPose.get<0>() ),2) + pow( ( goalPose.get<1>() - currRobotPose.get<1>() ),2) ) > 0.001 );

}
void testVel(Vector2D speed,double yaw,Robot& robot,time_t testTime){
#ifdef GAZEBO
//	time_t startTime=time(NULL);
	speed=speed*Config::getInstance().getSpeedFactor();
	std::cout<<"start test with vx="<<speed.x<<" vy="<<speed.y<<" w="<<yaw<<"test time "<<testTime<<std::endl;
	Pose startPosition;
	SimControl::getInstance().getModelPos(robot.getRobotName(),startPosition);
	Pose prevPosition=startPosition;
	Pose currPosition=startPosition;
	double currSimTime,prevSimTime;
	double vx=0,vy=0;
	prevSimTime=SimControl::getInstance().getSimTime();

	//sprawdz czy robot przejechal 1 metr
	//while(time(NULL)-startTime<5){
	while( startPosition.distance(currPosition) < 1.0 ){
		//currSimTime=SimControl::getInstance().getSimTime();
		robot.setRelativeSpeed(speed,yaw);
		//Pose position;
		//std::map<std::string,Pose > positions;
		SimControl::getInstance().getModelPos(robot.getRobotName(),currPosition);
		currSimTime=SimControl::getInstance().getSimTime();

		vx=(currPosition.get<0>()-prevPosition.get<0>())/(currSimTime-prevSimTime);
		vy=(currPosition.get<1>()-prevPosition.get<1>())/(currSimTime-prevSimTime);
		//Logger::getInstance().LogToFile(DBG,"vx=%f vy=%f simTime=%f",vx,vy,currSimTime);

		double rot=currPosition.get<2>();
		//macierz obrotu os OY na wprost robota
		RotationMatrix rm(rot);
		Vector2D tmp1(robot.getVelocity().first);

		Vector2D tmp2(vx,vy);

		Vector2D t=rm.Inverse()*tmp2;
		//Vector2D tmp2=rmY.Inverse()*tmp1;
		///Vector2D targetPosition=rmX.Inverse()*(goToPosition-currentPosition);
		std::cout<<"calculated robotVelocity in robot coordinates"<<t<<std::endl;
		std::cout<<"calculated robotVelocity in global coordinates"<<tmp2<<std::endl;
		std::cout<<"robotVelocity from gazebo in global coordinates"<<tmp1<<std::endl;
		prevSimTime=currSimTime;
		prevPosition=currPosition;

		//SimControl::getInstance().getAllPos(positions);
		usleep(100000);//100ms
	}
    std::cout<<"HAMOWANIE"<<std::endl;
	robot.setRelativeSpeed(Vector2D(0.0,0.0),0);
	sleep(10);
	getchar();
	//SimControl::getInstance().restart();
	//usleep(1000000);
#endif
	return;
}

double calculateAngularVel(GameState & gameState,Robot::robotID robotID, Pose targetPosition){
    //static GameState oldGameState;
    static double oldTetaCel;
    double rotacjaDocelowa=atan2(targetPosition.get<0>(),targetPosition.get<2>());
    double Ker=0.5;
    double Ko=20;
    double currGlobalRobotRot=gameState.getRobotPos( robotID ).get<2>();
    //macierz obrotu os OY na wprost robota
    RotationMatrix rmY(currGlobalRobotRot);
    //macierz obrotu os OY nw wprost robota
    //RotationMatrix rmY(-M_PI/2);
    //pozycja robota w ukladzie wsp zw z plansza
    //Vector2D currentPosition=Videoserver::data.getPosition(this->robotName);
    Pose currRobotPose=gameState.getRobotPos( robotID );
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
void checkAcceleration(const Vector2D& speed_,Robot& robot){
	std::cout<<"checkAcceleration tests speed"<<speed_<<std::endl;

	Vector2D speed(speed_);
	GameStatePtr gameState(new GameState());
	double deltaTime=0;
	double currSimTime=0 ,prevSimTime=Videoserver::getInstance().updateGameState(gameState);

	bool exit_=false;
	bool brake=false;
	Vector2D maxSpeed;

	double maxLength = speed.length()*0.9;

	robot.setRelativeSpeed(speed,0);
	while(true){
	    currSimTime=Videoserver::getInstance().updateGameState(gameState);

        if( prevSimTime <  currSimTime ){
        	Pose currPose = gameState->getRobotPos( robot.getRobotID() );
        	Pose targetPosition(currPose.get<0>()+speed.x,currPose.get<1>()+speed.y,0);
        	Vector2D currspeed = (*gameState).getRobotVelocity( robot.getRobotID() );

        	if(!brake)
        		robot.setRelativeSpeed(speed,0);
        	else
        		robot.stop();
        	//robot.setRelativeSpeed(speed,
        	//		calculateAngularVel( *gameState, robot.getRobotID(), targetPosition)
        	//		);

        	std::cout<<"currSimTime"<<currSimTime<<" currspeed "<<currspeed<<std::endl;
            deltaTime+=(currSimTime - prevSimTime);
            prevSimTime=currSimTime;

            //robot.setSpeed(speed,0);

            //(*gameState).updateRobotVel(robot.getRobotName(),robot.getVelocity() );

            if( ( ( currspeed.length() >= maxLength  ) && !brake )
                    || ( (fabs( currspeed.length() ) <0.01 )  && brake ) )
            {
                if(!brake){
                    std::cout<<"############ velocity "<<currspeed<<" delta Time "<<deltaTime<<" acc="
                        <<currspeed.length()/deltaTime<<std::endl;
                    maxSpeed=currspeed;
                    //exit(0);
                }
                else{
                    std::cout<<"############ velocity "<<(*gameState).getRobotVelocity( robot.getRobotID() )<<" delta Time "<<deltaTime<<" dcc="
                                <<maxSpeed.length()/deltaTime<<std::endl;
                }
                //speed=Vector2D(0,0);
                speed.x = -speed_.x;
                speed.y = -speed_.y;
                deltaTime=0;
                robot.setRelativeSpeed(speed,0);
                if(exit_)
                    break;
                exit_=true;
                brake=true;
            }
        }
	}
	robot.setRelativeSpeed(Vector2D(0,0),0);

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
	Videoserver::getInstance().start(NULL);

    while(true){
        pthread_create(&red0, &attr, testTask, (void *) &redRobot0);


        //pthread_create(&red1, &attr, testTask, (void *) &redRobot1);
        //pthread_create(&red2, &attr, testTask, (void *) &redRobot2);
        //pthread_create(&blue0, &attr,testTask, (void *) &blueRobot0);
       //pthread_create(&blue1,the gossip heavy cross &attr, testTask, (void *) &blueRobot1);
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

	//GoToPose goToPose( (*gameState).getBallPos(),robot);
	//GoToPose goToPose( Pose(6.0,4.0,0.0),robot);
	//goToPose.execute();
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
    bool serialize=true;
	TestRRT testRRT(&redRobot0,Pose(4.0,6.0,0),serialize);
    testRRT.joinThread();

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


    bool serialize=Config::getInstance().isDebugMode();
	TestRRT testRRTred0(&redRobot0,Pose(5.5,2.5,0),serialize);


	TestRRT testRRTred1(&redRobot1,Pose(1.5,2.5,0),serialize);
	TestRRT testRRTred2(&redRobot2,Pose(5.3,0.5,0),serialize);

	TestRRT testRRTblue0(&blueRobot0,Pose(1.3,0.5,0),serialize);
	TestRRT testRRTblue1(&blueRobot1,Pose(2.3,1.5,0),serialize);
	TestRRT testRRTblue2(&blueRobot2,Pose(2.6,0.5,0),serialize);

	testRRTred0.joinThread();
	testRRTred1.joinThread();
	testRRTred2.joinThread();

	testRRTblue0.joinThread();
	testRRTblue1.joinThread();
	testRRTblue2.joinThread();

	std::cout<<"exit from testMultiRRTThread"<<std::endl;
}

void testDribbler(Robot& robot){

#ifdef GAZEBO
	GameStatePtr gameState(new GameState());
	Videoserver::getInstance().start(NULL);
	Videoserver::getInstance().updateGameState(gameState);
    double dist;
    while( ( dist= (*gameState).getBallPos().distance(gameState->getRobotPos(robot.getRobotID() ) ) ) >
            ( Config::getInstance().getRobotMainCylinderRadious() + 0.04 ) ){
        std::cout<<"dist "<<dist<<std::endl;
        GoToPose goToPose( (*gameState).getBallPos(),&robot);
        if( goToPose.execute(NULL) == false)
            SimControl::getInstance().restart();
        Videoserver::getInstance().updateGameState(gameState);
    };
#endif

/*
    std::cout<<"try to correct robot rotation"<<std::endl;
    //ustaw rotacjeboost::weak_ptr<Task*>(this)
    double lastUpdateTime=0;
    double currUpdateTime=0;
    double maxAngularVel=3.14;//rad/sek
    Pose targetPosition=(*gameState).getBallPos();
    double teta_cel=1;
    while( fabs(teta_cel) > 0.01){
        if( (currUpdateTime=Videoserver::getInstance().updateGameState(gameState))>lastUpdateTime ){
            lastUpdateTime=currUpdateTime;
            double rot=(*gameState).getRobotPos( robot.getRobotName()).get<2>() ;
            //macierz obrotu os OX na wprost robota
            RotationMatrix rmX(rot);
            //macierz obrotu os OY nw wprost robota
            //RotationMatrix rmY(-M_PI/2);
            //pozycja robota w ukladzie wsp zw z plansza
            //Vector2D currentPosition=Videoserver::data.getPosition(this->robotName);
            Pose currRobotPose=(*gameState).getRobotPos( robot.getRobotName() );
            //pozycja celu w ukladzie wsp zwiazanych z robotem
            Pose reltargetPose=targetPosition.transform(currRobotPose.getPosition(),rmX);
            //targetPosition=rmX.Inverse()*(goToPosition-currentPosition);
            //rotacja do celu
            teta_cel=atan2( (reltargetPose.get<1>()) , (reltargetPose.get<0>()));

            robot.setRelativeSpeed(Vector2D(0,0),teta_cel > maxAngularVel ? maxAngularVel : teta_cel);
            //targetPosition = rmY.Inverse() * targetPosition;
        }
    }

    robot.setRelativeSpeed(Vector2D(0,0),0);
	//sleep(1);
	*/
}

void testShootTactics(void * arg){
#ifdef GAZEBO
    std::cout<<" start tactic test "<<std::endl;
    //warunek na sprawdzenie czy pilka jest w posiadaniu robota
    //    while( ( dist = ballPose.distance( gameState->getRobotPos( redRobot0.getRobotID() ) ) ) >
    //               ( Config::getInstance().getRobotMainCylinderRadious() + 0.04 ) ){
	Robot* redRobot0 = reinterpret_cast<Robot*>(arg);//(std::string("red0"),ifaceName);

	//jedz do pilki
	GameStatePtr gameState(new GameState());
	Videoserver::getInstance().updateGameState(gameState);

    Task::status taskStatus = Task::not_completed;
    Task* task = ( new GoToPose( (*gameState).getBallPos(), redRobot0 )  );
    Task* newTask;
    while( taskStatus != Task::ok ){

    	//transition to next TASK
    	newTask = task->nextTask();

    	if(newTask){
    		delete task;
    		task=newTask;
    		newTask=NULL;
    	}
    	//comand generation
    	taskStatus = task->execute(NULL,1);


        if( taskStatus == Task::collision )
            SimControl::getInstance().restart();

        Videoserver::getInstance().updateGameState(gameState);
    };

    redRobot0->stop();

    std::cout<<" robot have ball "<<EvaluationModule::getInstance().isRobotOwnedBall(*redRobot0)<<std::endl;

    exit(0);

    AbstractTactic * shootTactic= new ShootTactic(*redRobot0);
    shootTactic->execute(NULL);
    shootTactic->join();
    sleep(1);
    redRobot0->stop();

    SimControl::getInstance().restart();
    sleep(1);

#endif

}

void testKick(){
#ifdef GAZEBO
    std::cout<<" start tactic test "<<std::endl;

	Videoserver::getInstance().start(NULL);

	Robot redRobot0(std::string("red0"),ifaceName);


	//jedz do pilki
	GameStatePtr gameState(new GameState());
	Videoserver::getInstance().start(NULL);
	Videoserver::getInstance().updateGameState(gameState);

    TaskSharedPtr task( new KickBall(&redRobot0, 0) );
    task->execute( NULL );

#endif
}
