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
#include "../Task/GetBall.h"
#include "../Task/Task.h"
#include "../Task/KickBall.h"

#include "../TestRRT/TestRRT.h"
#include "../Tactics/Tactic.h"
#include "../Tactics/ShootTactic.h"
#include "../Tactics/DefendLine.h"
#include "../Tactics/Receive_pass.h"
#include "../Tactics/Pass.h"
#include "../Plays/ObstaclesPlay.h"
#include "../Plays/Experiment1.h"
#include "../Plays/Experiment2.h"


const std::string ifaceName="::position_iface";


void testRound(){
	//while( haveBall && (  t.get<1>() < 0 ) ){
	//Vector2D oy( 0.0,1.0 );
	//double angle = oy.angleTo( t.getPosition( ) );
	double lastSimTime =0;
	double currSimTime = 0;
	Videoserver& video = Videoserver::getInstance();
	GameStatePtr currGameState(new GameState() );
	Robot robot(std::string("blue0"),ifaceName);
	currSimTime = video.updateGameState(currGameState);
	Pose currRobotPose=(*currGameState).getRobotPos( robot.getRobotID() );
	Pose nextRobotPose(currRobotPose.get<0>(),currRobotPose.get<1>()-1.0,currRobotPose.get<2>());
	while(1){
		while( ( lastSimTime - currSimTime ) >= 0 ){
			currSimTime = video.updateGameState(currGameState);
		}
		currRobotPose=(*currGameState).getRobotPos( robot.getRobotID() );
		double robotRotation = currRobotPose.get<2>();
		RotationMatrix rm(robotRotation);
		Pose t = nextRobotPose.transform( currRobotPose.getPosition() , rm);

		double angle = convertAnglePI(atan2(t.get<1>(),t.get<0>()) -M_PI/2.0);
		//LOG_FATAL( log, "currRobotPose "<< currRobotPose <<" globalNextPose "<<nextRobotPose << " relative nextRobotPose  "<<t<<" angle "<<angle );
		double maxW = fabs(angle)/video.getUpdateDeltaTime() ;//> 2*M_PI ? 2*M_PI : fabs(angle)/video.getUpdateDeltaTime() ;
		double ball_radious = 0.02;

		//double angle = t.getPosition().angleTo( Vector2D( 0.0,1.0 ) );

		boost::tuple< double, double, double > vel = calculateCurwatureVelocity( ball_radious*sgn(angle) , maxW );
		Vector2D v = Vector2D( vel.get<0>(), vel.get<1>() );
		double w = vel.get<2>();
		robot.setRelativeSpeed( v, w );

		lastSimTime = currSimTime;
		//currSimTime = video.updateGameState(currGameState) ;
		currRobotPose=(*currGameState).getRobotPos( robot.getRobotID() );
		robotRotation = currRobotPose.get<2>();
		rm = RotationMatrix(robotRotation);
		//t = nextRobotPose.transform( currRobotPose.getPosition() , rm);
		//haveBall = this->evaluationModule.isRobotOwnedBall( this->robot );

		//LOG_FATAL( log, "haveBall "<<haveBall );
	}
}

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
	//goalPositions.push_back(Vector2D(-1,0));
	//goalPositions.push_back(Vector2D(0,1));
	//goalPositions.push_back(Vector2D(0,-1));
	video.updateGameState(gameState);

	std::vector<Vector2D>::iterator ii=goalPositions.begin();

	Vector2D v(0,0);
	double oldAlfaToCel = 0;

	double currSimTime = 0;
	double prevSimTime = 0;

	const double ROTATION_HYSTERESIS = 0.01;//0.02 radiana
	Pose currRobotPose = gameState->getRobotPos( robot.getRobotID() );
	for(;ii!=goalPositions.end();ii++){
		double w =1;
		Vector2D goToPosition = currRobotPose.getPosition() + *ii;
		Pose globalTargetPose( goToPosition,0 );

		Vector2D goToPosition2 = currRobotPose.getPosition() + (*ii) + Vector2D(0.01,0.01);
		Pose globalTargetPose2( goToPosition2,0 );

		bool globalTargetPose1 = true;

		while( fabs(w) > 0.01 ){

			currSimTime = video.updateGameState( gameState );

			if( currSimTime > prevSimTime ){
				currRobotPose = gameState->getRobotPos( robot.getRobotID() );
				prevSimTime = currSimTime;


				std::cout<<"rotate to position to position "<<*ii<<std::endl;
				(*gameState).updateRobotVel( robot.getRobotID(),robot.getGlobalVelocity( ) );
				deltaTime=video.getUpdateDeltaTime();

				double rot=(*gameState).getRobotPos( robot.getRobotID() ).get<2>() ;
				{
					double Ker=0.5;
					double Ko=20;
					double currGlobalRobotRot = currRobotPose.get<2>();

					// ten kawalek kodu wyznacza kat o jaki robot musi sie obrocic zeby byc skierowanym na cel
					RotationMatrix rm0(0);
					Pose reltargetPose_;
					if(globalTargetPose1){
						reltargetPose_ = globalTargetPose.transform( currRobotPose.getPosition(),rm0 );
						globalTargetPose1 = false;
					}
					else{
						reltargetPose_ = globalTargetPose2.transform( currRobotPose.getPosition(),rm0 );
						globalTargetPose1 = true;
					}

					Pose reltargetPose = reltargetPose_*100;
					double rotacjaDocelowa=-atan2(reltargetPose.get<0>(),reltargetPose.get<1>()) ;
					std::cout<<" rotacjaDocelowa "<< rotacjaDocelowa <<std::endl;
					assert( fabs(rotacjaDocelowa) < M_PI);

					//obrot jaki trzeba wykonac w biezacym kroku
					double currAlfaToCel = convertAnglePI( rotacjaDocelowa - currGlobalRobotRot );///convertAnglePI( currGlobalRobotRot - rotacjaDocelowa  );
					std::cout<<"obrot jaki trzeba wykonac w biezacym kroku "<<currAlfaToCel<<std::endl;
					double angularVel=Ko*(  currAlfaToCel  )+ Ker*( convertAnglePI( oldAlfaToCel - currAlfaToCel) );

					w = fabs(angularVel) > M_PI ? M_PI * sgn(angularVel) : angularVel;

					oldAlfaToCel=currAlfaToCel;

				}
				//w = robot.calculateAngularVel( gameState->getRobotPos( robot.getRobotID() ), Pose( goToPosition,0 ) );

				std::cout<<"rotacja robota "<<rot <<" angular velocity"<< w <<std::endl;
				robot.setGlobalSpeed(v,w,rot);
				usleep(10000);//10ms
			}
		}
		std::cout<<"####################### FINISH ############################### " <<std::endl;
		sleep(1);
	}
}
void testMotion(const Pose goalPose,Videoserver & video,Robot& robot){
	GameStatePtr gameState(new GameState());
	double deltaTime=0;
	Pose currRobotPose;
	video.updateGameState(gameState);
	Vector2D speed;
	video.updateGameState(gameState);

	std::cout<<"######################GoalPose"<<goalPose<<"#####################"<<std::endl;

	do {
		video.updateGameState(gameState);
		(*gameState).updateRobotVel( robot.getRobotID() ,robot.getGlobalVelocity( ) );
		deltaTime=video.getUpdateDeltaTime();

		//double rot=(*gameState).getRobotPos( robot.getRobotID() ).get<2>() ;
		std::cout<<"robot speed from gazebo"<<(*gameState).getRobotGlobalVelocity( robot.getRobotID() )
				<<" w "<<(*gameState).getRobotAngularVelocity( robot.getRobotID() )<<std::endl;
		//macierz obrotu os OY na wprost robota
		//RotationMatrix rmY(rot);

		currRobotPose=(*gameState).getRobotPos( robot.getRobotID() );
		//pozycja robota w ukladzie wsp zw z plansza
		Vector2D currentAbsPosition( currRobotPose.get<0>(),currRobotPose.get<1>() );
		Vector2D goToAbsPosition(goalPose.get<0>(),goalPose.get<1>());

		//pozycja celu w ukladzie wsp zwiazanych z robotem
		//Vector2D targetRelPosition=rmY.Inverse()*(goToAbsPosition-currentAbsPosition);

		Vector2D robotVel=(*gameState).getRobotGlobalVelocity( robot.getRobotID() );
		bool haveBall = false;
		double w = robot.calculateAngularVel( gameState->getRobotPos( robot.getRobotID() ), goalPose ,(*gameState).getSimTime(), haveBall);
		//speed=calculateVelocity( robotVel, Pose(targetRelPosition.x,targetRelPosition.y,0));
		speed=robot.calculateVelocity( robotVel,currRobotPose, goalPose);

		std::cout<<"set speed "<<speed<<" w "<<w<<std::endl;

		//robot.setRelativeSpeed(speed,w);
		robot.setGlobalSpeed(speed,w, currRobotPose.get<2>());
	}while( sqrt( pow( ( goalPose.get<0>()-currRobotPose.get<0>() ),2) + pow( ( goalPose.get<1>() - currRobotPose.get<1>() ),2) ) > 0.001 );

}
void testVel(Vector2D speed,double yaw,Robot& robot,time_t testTime){
#ifdef GAZEBO

	//Videoserver::getInstance().stop();
	//Videoserver::getInstance().join();


	sleep(1);
	speed=speed*Config::getInstance().getSpeedFactor();
	std::cout<<"start test with vx="<<speed.x<<" vy="<<speed.y<<" w="<<yaw<<"test time "<<testTime<<std::endl;
	Pose startPosition;
	SimControl::getInstance().getModelPos(robot.getRobotName(),startPosition);
	Pose prevPosition=startPosition;
	Pose currPosition=startPosition;
	double currSimTime,prevSimTime;
	double vx=0,vy=0,w=0;
	prevSimTime=SimControl::getInstance().getSimTime();

	//sprawdz czy robot przejechal 1 metr
	//while(time(NULL)-startTime<5){


	while( startPosition.distance(currPosition) < 3.0 ){
		currSimTime=SimControl::getInstance().getModelPos(robot.getRobotName(),currPosition);

		robot.setGlobalSpeed(speed,yaw, currPosition.get<2>());

		std::cout<<"current robot position "<<currPosition<<std::endl;
		vx=(currPosition.get<0>()-prevPosition.get<0>())/(currSimTime-prevSimTime);
		vy=(currPosition.get<1>()-prevPosition.get<1>())/(currSimTime-prevSimTime);
		double rot_radians = ( currPosition.get<2>() * 180.0 ) /M_PI;
		double deltaAlfa = convertAnglePI( currPosition.get<2>() - prevPosition.get<2>() );
		std::cout<<"current rot in radians"<< currPosition.get<2>()<<" in degrees "<<rot_radians<<" currSimTime "<<currSimTime <<" deltaAlfa "<<deltaAlfa<<" deltaSimTime"<<currSimTime-prevSimTime<<std::endl;
		w=( deltaAlfa  )/( currSimTime-prevSimTime );

		double rot=currPosition.get<2>();
		//macierz obrotu os OY na wprost robota
		RotationMatrix rm(rot);
		Vector2D relSpeed(robot.getGlobalVelocity().first);
		double ang_vel = robot.getGlobalVelocity().second;

		Vector2D tmp2(vx,vy);

		Vector2D t=rm.Inverse()*tmp2;
		//Vector2D tmp2=rmY.Inverse()*tmp1;
		///Vector2D targetPosition=rmX.Inverse()*(goToPosition-currentPosition);
		std::cout<<"calculated relative robot velocity "<<t<<std::endl;
		std::cout<<"calculated global robot velocity "<<tmp2<<" angular velocity "<<w<<std::endl;

		std::cout<<"global robot velocity from gazebo"<<relSpeed<<" angular velocity "<<ang_vel<<std::endl;
		//RotationMatrix rm2(-rot);
		//Vector2D rel = relSpeed.rotate(rot);
		//std::cout<<"global robot velocity from gazebo"<<relSpeed.rotate(rot)<<" angular velocity "<<ang_vel<<std::endl;

		prevSimTime=currSimTime;
		prevPosition=currPosition;

		robot.setGlobalSpeed(speed,0, currPosition.get<2>());
		//SimControl::getInstance().getAllPos(positions);
		usleep(500000);//100ms
	}
    std::cout<<"HAMOWANIE"<<std::endl;
    robot.stop();
	//robot.setRelativeSpeed(Vector2D(0.0,0.0),0);
	sleep(1);
	//getchar();
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

	//robot.setRelativeSpeed(speed,0);
	while(true){
	    currSimTime=Videoserver::getInstance().updateGameState(gameState);

        if( prevSimTime <  currSimTime ){
        	Pose currPose = gameState->getRobotPos( robot.getRobotID() );
        	Pose targetPosition(currPose.get<0>()+speed.x,currPose.get<1>()+speed.y,0);
        	Vector2D currspeed = (*gameState).getRobotGlobalVelocity( robot.getRobotID() );

        	if(!brake){
        		//robot.setRelativeSpeed(speed,0);
        		robot.setGlobalSpeed( speed,0, currPose.get<2>() );
        	}
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
                    std::cout<<"############ velocity "<<(*gameState).getRobotGlobalVelocity( robot.getRobotID() )<<" delta Time "<<deltaTime<<" dcc="
                                <<maxSpeed.length()/deltaTime<<std::endl;
                }
                //speed=Vector2D(0,0);
                speed.x = -speed_.x;
                speed.y = -speed_.y;
                deltaTime=0;
                robot.setGlobalSpeed( speed,0, currPose.get<2>() );
                //robot.setRelativeSpeed(speed,0);
                if(exit_)
                    break;
                exit_=true;
                brake=true;
            }
        }
	}
	robot.stop();
	//robot.setRelativeSpeed(Vector2D(0,0),0);

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
			//SimControl::getInstance().restart();
			//TODO: poczekaj az swiat sie zrestartuje!!!!!
		#endif

   }
}

void * testTask(void * arg){
    //std::cout<<"start testTask"<<std::endl;

	Robot * robot=reinterpret_cast<Robot *>(arg);
	GameStatePtr gameState(new GameState());
	Videoserver::getInstance().updateGameState(gameState);

	GoToPose goToPose( (*gameState).getBallPos().getPosition(),robot);
	//GoToPose goToPose( Pose(6.0,4.0,0.0),robot);
	goToPose.execute(NULL,-1);
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


 //   Videoserver::getInstance().start(NULL);

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
    /*
    while( ( dist= (*gameState).getBallPos().distance(gameState->getRobotPos(robot.getRobotID() ) ) ) >
            ( Config::getInstance().getRobotMainCylinderRadious() + 0.04 ) ){
        std::cout<<"dist "<<dist<<std::endl;
        GoToPose goToPose( (*gameState).getBallPos().getPosition(),&robot);
        if( goToPose.execute(NULL) == false)
            SimControl::getInstance().restart();
        Videoserver::getInstance().updateGameState(gameState);
    };
    */
    int steps = 1;
    Task* task = NULL;
    Task* nextTask_ = NULL;
    EvaluationModule::ballState bs;
	EvaluationModule& evaluation=EvaluationModule::getInstance();
    bool gettingBall = false;
	while(true){

    	bs = evaluation.getBallState(  robot.getRobotID( ) );
    	if( bs == EvaluationModule::free ){
    		if( !gettingBall ){
    			gettingBall = true;
    			if( task ){
    				delete task;
    				task = NULL;
    			}
    			task = new GetBall( &robot );
    		}
    	}
    	else if( bs == EvaluationModule::mine ){
    		SimControl::getInstance().restart();
    		continue;
    		/*
    		if( gettingBall ){
    			gettingBall =  false;
    			//odleglosc od pkt docelowego przy jakiej stwierdzamy ze robot dojechal do celu
    			const double minDist = 0.1;
    			if( task ){
    				delete task;
    				task = NULL;
    			}
    			task = new GoToPose( Config::getInstance().field.BOTTOM_GOAL_MID_POSITION, &robot,  minDist);
    		}
    		*/
    	}
    	else{
    		SimControl::getInstance().restart();
    		continue;
    	}
    	nextTask_ = task->nextTask();
    	if(nextTask_){
    		if( task ){
    			delete task;
    			task =nextTask_;
    		}
    	}
    	task->execute(NULL,steps);
    	Videoserver::getInstance().updateGameState(gameState);
    }
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

void testNaviFuncion(void * arg){
	std::cout<<"start testNaviFuncion"<<std::endl;
	GameStatePtr gameState(new GameState());
	Videoserver::getInstance().start(NULL);
	Videoserver::getInstance().updateGameState(gameState);
	Robot* red0 = reinterpret_cast<Robot*>(arg);
    double dist;
    red0->disperse(0.05);
	std::cout<<"exit from testNaviFuncion"<<std::endl;
/*
    while( ( dist= (*gameState).getBallPos().distance(gameState->getRobotPos(robot.getRobotID() ) ) ) >
            ( Config::getInstance().getRobotMainCylinderRadious() + 0.04 ) ){
        std::cout<<"dist "<<dist<<std::endl;
        GoToPose goToPose( (*gameState).getBallPos().getPosition(),&robot);
        if( goToPose.execute(NULL) == false)
            SimControl::getInstance().restart();
        Videoserver::getInstance().updateGameState(gameState);
    };
*/

    /*
    int steps = 1;
    Task* task = NULL;
    Task* nextTask_ = NULL;
    EvaluationModule::ballState bs;
	EvaluationModule& evaluation=EvaluationModule::getInstance();
    bool gettingBall = false;
	while(true){

    	bs = evaluation.getBallState(  robot.getRobotID( ) );
    	if( bs == EvaluationModule::free ){
    		if( !gettingBall ){
    			gettingBall = true;
    			if( task ){
    				delete task;
    				task = NULL;
    			}
    			task = new GetBall( &robot );
    		}
    	}
    	else if( bs == EvaluationModule::mine ){
    		if( gettingBall ){
    			gettingBall =  false;
    			//odleglosc od pkt docelowego przy jakiej stwierdzamy ze robot dojechal do celu
    			const double minDist = 0.1;
    			if( task ){
    				delete task;
    				task = NULL;
    			}
    			task = new GoToPose( Config::getInstance().field.BOTTOM_GOAL_MID_POSITION, &robot,  minDist);
    		}
    	}
    	else{
    		SimControl::getInstance().restart();
    		continue;
    	}
    	nextTask_ = task->nextTask();
    	if(nextTask_){
    		if( task ){
    			delete task;
    			task =nextTask_;
    		}
    	}
    	task->execute(NULL,steps);
    	Videoserver::getInstance().updateGameState(gameState);
    }
    */
}
void testPassTacticFunc(void * arg){
#ifdef GAZEBO

    std::cout<<" start pass test "<<std::endl;
    //warunek na sprawdzenie czy pilka jest w posiadaniu robota
    //    while( ( dist = ballPose.distance( gameState->getRobotPos( redRobot0.getRobotID() ) ) ) >
    //               ( Config::getInstance().getRobotMainCylinderRadious() + 0.04 ) ){
	Robot* blue0 = reinterpret_cast<Robot*>(arg);//(std::string("red0"),ifaceName);

	static Robot blue1(std::string("blue1"),ifaceName);
	//jedz do pilki
	//GameStatePtr gameState(new GameState());
	//Videoserver::getInstance().updateGameState(gameState);

	Robot *pass = blue0;
	Robot *recv = &blue1;

	GameStatePtr gameState(new GameState());
	Videoserver::getInstance().updateGameState(gameState);

	Pose start0 = gameState->getRobotPos(blue0->getRobotID());
	Pose start1 = gameState->getRobotPos(blue1.getRobotID());

	while(true){

		Tactic * receive_pass= new Receive_pass(*recv);
		receive_pass->start( );

		Tactic * pass_tactic= new Pass(*pass, recv->getRobotID());

		pass_tactic->start( );

		pass_tactic->waitForFinish();
		std::cout<<" after join pass_tactic "<<std::endl;
//		receive_pass->join();

		receive_pass->stopTactic();
		receive_pass->waitForFinish();
		std::cout<<" after join receive_pass "<<std::endl;

		delete receive_pass;
		delete pass_tactic;

		sleep(5);
		SimControl::getInstance().setSimPos( "ball", Config::getInstance().field.FIELD_MIDDLE_POSE );
		SimControl::getInstance().setSimPos( "blue0", start0 );
		SimControl::getInstance().setSimPos( "blue1", start1 );

		Robot * tmp = pass;
		pass = recv;
		recv = tmp;
	}

#endif //GAZEBO
}

void testShootTacticFunc(void * arg){
#ifdef GAZEBO
    std::cout<<" start shoot tactic test "<<std::endl;
    //warunek na sprawdzenie czy pilka jest w posiadaniu robota
    //    while( ( dist = ballPose.distance( gameState->getRobotPos( redRobot0.getRobotID() ) ) ) >
    //               ( Config::getInstance().getRobotMainCylinderRadious() + 0.04 ) ){
	Robot* robot = reinterpret_cast<Robot*>(arg);//(std::string("red0"),ifaceName);

	//jedz do pilki
	GameStatePtr gameState(new GameState());
	Videoserver::getInstance().updateGameState(gameState);

	static Robot blue0(std::string("blue0"),ifaceName);
	//Pose p1 = Pose( *Config::getInstance().field.BLUE_GOAL_LEFT_CORNER, 0 );
	//Pose p2 = Pose( *Config::getInstance().field.BLUE_GOAL_RIGHT_CORNER, 0 );

	Pose p1 = Pose( Videoserver::getBlueGoalLeftCornerPosition(), 0 );
	Pose p2 = Pose( Videoserver::getBlueGoalRightCornerPosition(), 0 );

	double maxDist = 1.0;

	Tactic * defendLine= new DefendLine( blue0, p1.getPosition(),p2.getPosition(), maxDist);
	defendLine->start();

	static Robot blue1(std::string("blue1"),ifaceName);
	//Vector2D p1(Config::getInstance().field.TOP_GOAL_LEFT_CORNER.x,Config::getInstance().field.TOP_GOAL_LEFT_CORNER.y -
	p1 = gameState->getRobotPos( blue1.getRobotID() );
	p1=p1+Pose(1.0,1.0,0);
	p2 = gameState->getRobotPos( blue1.getRobotID() );
	p2=p2-Pose(1.0,1.0,0);
	maxDist = 1.0;

	Tactic * defendLine1= new DefendLine( blue1, p1.getPosition(),p2.getPosition(), maxDist);
	defendLine1->start();

/*
	static Robot blue2(std::string("blue2"),ifaceName);
	//Vector2D p1(Config::getInstance().field.TOP_GOAL_LEFT_CORNER.x,Config::getInstance().field.TOP_GOAL_LEFT_CORNER.y -
	p1 = gameState->getRobotPos( blue2.getRobotID() );
	p1=p1+Pose(1.0,1.0,0);
	p2 = gameState->getRobotPos( blue2.getRobotID() );
	p2=p2-Pose(1.0,1.0,0);
	maxDist = 1.0;

	Tactic * defendLine2= new DefendLine( blue2, p1.getPosition(),p2.getPosition(), maxDist);
	defendLine2->start();
*/
    Tactic * shootTactic= new ShootTactic(*robot);
    //shootTactic->execute(NULL);
    shootTactic->start( );
    shootTactic->waitForFinish();
    sleep(1);
    robot->stop();

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

    //TaskSharedPtr task( new KickBall(&redRobot0, 0) );
    //task->execute( NULL );

#endif
}

void run_experiment_1(){

	boost::shared_ptr<Play> redPlay ( new ObstaclesPlay("red", 6) );
	redPlay->execute();
	log4cxx::LoggerPtr log = getLoggerPtr ("app_debug");
	for(int i=0;i<20;i++){
		std::ostringstream ois;
		ois<<i;

		LOG_INFO(log, "starting experiment 1" );

		struct timespec req;
		req.tv_sec = 0;
		req.tv_nsec = 10000000;//10ms;
		struct timespec rem;
		bzero(&rem, sizeof(rem) );

		//RefereeClient & referee = RefereeClient::getInstance();


		boost::shared_ptr<Play> bluePlay ( new Experiment_1("blue",3, ois.str() ) );

		bluePlay->execute();


		//sleep(5);
		//bluePlay->stop();
		GameStatePtr gameState( new GameState() );
		while( !bluePlay->isFinished() ){
			//bluePlay->waitForFinish();
			Videoserver::getInstance().updateGameState( gameState );
			EvaluationModule::ballState bs = EvaluationModule::getInstance().getBallState(Robot::blue0);
			if( bs == EvaluationModule::out || bs == EvaluationModule::in_goal ){
				SimControl::getInstance().setSimPos("ball",Config::getInstance().field.FIELD_MIDDLE_POSE);
			}
			sleep(1);
		}
		LOG_FATAL(log, "blue Play finished" );
		//redPlay->stop();
		//redPlay->waitForFinish();
		//LOG_FATAL(log, "red Play finished" );
		SimControl::getInstance().restart();
		SimControl::getInstance().moveBall( Config::getInstance().field.FIELD_MIDDLE_POSE );

	}
	redPlay->stop();
	redPlay->waitForFinish();

	LOG_INFO(log, "end from experiment 1" );
}

void run_experiment_2(){

	log4cxx::LoggerPtr log = getLoggerPtr ("app_debug");

	LOG_INFO(log, "starting experiment 1" );

	struct timespec req;
	req.tv_sec = 0;
	req.tv_nsec = 10000000;//10ms;
	struct timespec rem;
	bzero(&rem, sizeof(rem) );

	while(true){
		Vector2D v( Config::getInstance().field.FIELD_MARIGIN*2.0,Config::getInstance().field.FIELD_MARIGIN*2.0);

		//Config::getInstance().field.
		SimControl::getInstance().moveBall( Pose( Config::getInstance().field.FIELD_TOP_RIGHT_CORNER-v,0) );
		boost::shared_ptr<Play> bluePlay ( new Experiment_2("blue",2 ) );

		bluePlay->execute();

		while( !bluePlay->isFinished() ){
			bluePlay->updateState( );
			usleep(1000);
		}
		sleep(2);
		//bluePlay->waitForFinish();
		LOG_FATAL(log, "blue Play finished" );
	}
	//SimControl::getInstance().restart();
	LOG_INFO(log, "end from experiment 2" );
}
