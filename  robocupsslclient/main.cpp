/*
 * main.cpp
 *
 *  Created on: 2009-05-07
 *      Author: maciek
 */

#include "RefereeClient/RefereeClient.h"
#include "EvaluationModule/EvaluationModule.h"
#include "Test/Test.h"

//#define TEST

#ifdef TEST
    #define BOOST_TEST_DYN_LINK

    #include <boost/test/unit_test.hpp>
    using namespace boost::unit_test_framework;
    #include <iostream>

    bool init_unit_test(){
        boost::unit_test::test_suite* ts1 = BOOST_TEST_SUITE( "test_suite1" );

        //boost::shared_ptr<RefereeClient> instance( new RefereeClient( ) );
        //framework::master_test_suite().add( BOOST_CLASS_TEST_CASE( &RefereeClient::testConnection, instance ) );
        //ts1->add( BOOST_CLASS_TEST_CASE( &RefereeClient::testConnection, instance ) );
        // ts1->add( BOOST_CLASS_TEST_CASE( &Videoserver::testVideoserver, Videoserver::getInstance() ) );

        //EvaluationModule m=EvaluationModule::getInstance();
        Pose currRobotPose(1,1,0);
        Pose targetPosition(3,3,0);

        EvaluationModule::getInstance().test(currRobotPose,targetPosition);

        currRobotPose=Pose(0,0,0);
		targetPosition=Pose(1.0,2,0);

		EvaluationModule::getInstance().test(currRobotPose,targetPosition);

	    currRobotPose=Pose(0,0,0);
		targetPosition=Pose(-1.0,2,0);

		EvaluationModule::getInstance().test(currRobotPose,targetPosition);
        //boost::shared_ptr<EvaluationModule> instance( &m );
        //ts1->add( BOOST_CLASS_TEST_CASE(&EvaluationModule::test,  instance ) );
        //boost::unit_test::framework::master_test_suite().add(ts1);
        return true;
    }

    int main(int argc, char *argv[]){
        return boost::unit_test::unit_test_main(&init_unit_test, argc, argv);
    }
#else

#include <iostream>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <time.h>
#include <limits>
#include <map>
#include <string>

#include "log4cxx/propertyconfigurator.h"

#include "Logger/Logger.h"
#include "Config/Config.h"
#include "Robot/Robot.h"
#include "my_signal.h"

bool run=true;

extern const std::string ifaceName;

int daemonInit ( void )	{
	pid_t pid;

	close (0); // Close stdin.
	close (1); // Close stdout.
	close (2); // Close stderr.

	if ( (pid = fork()) < 0 )	{ // Then fork() error.
		return -1;
	}
	else if ( pid != 0 )	{
		exit (0); // Parent process ends.
	}

	setsid(); // Session leader.

//	chdir("/");
//	umask(0);

	return 0;
}
namespace Tests{
 typedef   enum TestKind_{
            none=-1,
            position =0,
            velocity,
            RRTPlanner,
            multiRRTPlanner,
            acceleration,
            motion,
            rotation,
            taskScheduling,
            ballDribbling,
            testTactics
            } TestKind;
}

using namespace Tests;

int main(int argc, char*argv[],char *envp[]){

    //daemonInit ( );
    sys_catch_signals();
    // read config files
    //Init libxml
    xmlInitParser();
 //   Logger::getInstance().createLoggers();
 //   Logger::getInstance().disableCout();

// Logger::getInstance().enableCout();

    log4cxx::PropertyConfigurator::configure("log4cxx.properties");
    Config::getInstance().load("/home/maciek/codeblocks/magisterka/bin/Debug/config.xml");
    Config::getInstance().setTestMode(true);

    Tests::TestKind testKind=Tests::none;
    //std::cout<<"1e9 "<<1e9<<std::endl;
    //int b=2*(int)1e9;
    //int a=(int) (b- 1e9);
    //printf("%ld\n",a);
    //printf("%f\n",1e9);
    //std::cout<<"1E9 "<<1E9<<std::endl;

    //exit(0);

    if(argc>1){
        if(strncmp(argv[1],"test",4)==0)
            Config::getInstance().setTestMode(true);
        if(argc>2){
            if(strncmp(argv[2],"velocity",8)==0)
                testKind=Tests::velocity;
            if(strncmp(argv[2],"position",8)==0)
                testKind=Tests::position;
            if(strncmp(argv[2],"rrt",3)==0)
                testKind=Tests::RRTPlanner;
            if(strncmp(argv[2],"multirrt",8)==0)
                testKind=Tests::multiRRTPlanner;
            if(strncmp(argv[2],"acc",3)==0)
                testKind=Tests::acceleration;
            if(strncmp(argv[2],"motion",6)==0)
                testKind=Tests::motion;
            if(strncmp(argv[2],"rot",3)==0)
                testKind=Tests::rotation;
            if(strncmp(argv[2],"task",4)==0)
                testKind=Tests::taskScheduling;
            if(strncmp(argv[2],"dribbler",8)==0)
                testKind=Tests::ballDribbling;
            if(strncmp(argv[2],"tactic",6)==0)
                testKind=Tests::testTactics;
        }
        else{
            std::cout<<"missing param"<<std::endl;
            exit(0);
        }
    }

    if(Config::getInstance().isTestMode()){
        std::cout<<"starting test mode"<<std::endl;

        switch(testKind){
                case velocity :{
                    //std::cout<<"do you want to connect to gazebo server?"<<std::endl;
                    //getchar();
                    Robot testRobot(Config::getInstance().getTestModelName(),ifaceName);
                    std::vector<boost::tuple<double,double,double> > tests=Config::getInstance().getVelTests();
                    std::vector<boost::tuple<double,double,double> >::const_iterator ii=tests.begin();
                    for(;ii!=tests.end();ii++){
                        Vector2D vel(ii->get<0>(),ii->get<1>());
                        sleep(0);
                        testVel(vel,ii->get<2>(),testRobot,Config::getInstance().getTestEstimatedTime());
                    }
                }
                break;
                case position :{
                    Robot testRobot(Config::getInstance().getTestModelName(),ifaceName);
                    std::cout<<"starting position tests"<<std::endl;
                    std::vector<Pose> tests=Config::getInstance().getPoseTests();
                    std::vector<Pose>::const_iterator ii = tests.begin();
                    for(;ii!=tests.end();ii++){
                        Vector2D vel(ii->get<0>(),ii->get<1>());
                        testPose(testRobot,*ii);
                    }
                }
                break;
                case acceleration :{
                    Robot testRobot(Config::getInstance().getTestModelName(),ifaceName);
                    Robot redRobot1(std::string("red1"),ifaceName);
                    Robot redRobot2(std::string("red2"),ifaceName);

                    Robot blueRobot0(std::string("blue0"),ifaceName);
                    Robot blueRobot1(std::string("blue1"),ifaceName);
                    Robot blueRobot2(std::string("blue2"),ifaceName);

                    Videoserver::getInstance().start(NULL);

                    std::cout<<"starting acceleration tests"<<std::endl;


                    std::vector<boost::tuple<double,double,double> > tests=Config::getInstance().getVelTests();
                    std::vector<boost::tuple<double,double,double> >::const_iterator ii=tests.begin();
                    for(;ii!=tests.end();ii++){
                        Vector2D speed(ii->get<0>(),ii->get<1>());
                        //int i=30;
                        //while(i--)
                            checkAcceleration(speed,testRobot);
                        //testVel(vel,ii->get<2>(),testRobot,Config::getInstance().getTestEstimatedTime());
                    }


                }
                break;
                case motion: {
                    Robot testRobot(Config::getInstance().getTestModelName(),ifaceName);
                    std::cout<<"starting motion tests"<<std::endl;

                    testMotion(Config::getInstance().getRRTGoalPose(),Videoserver::getInstance(),testRobot);
                    testMotion(Pose (1,0,0),Videoserver::getInstance(),testRobot);
                    testMotion(Pose (1,1,0),Videoserver::getInstance(),testRobot);
                    testMotion(Pose (-1,-1,0),Videoserver::getInstance(),testRobot);
                }
                break;
                case rotation: {
                    Robot testRobot(Config::getInstance().getTestModelName(),ifaceName);
                    std::cout<<"starting rotation tests"<<std::endl;
                    testRotation(Videoserver::getInstance(),testRobot);
                }
                break;
                case RRTPlanner:{
                    std::cout<<"starting RRT tests"<<std::endl;
                    testSingleRRTThread();
                }
                break;
                case multiRRTPlanner:{
                    std::cout<<"starting multi robot RRT tests"<<std::endl;
                    testMultiRRTThread();
                }
                break;
                case  taskScheduling: {
                    std::cout<<"starting tasks tests"<<std::endl;
                    testTaskThread();
                }
                break;
                case ballDribbling :{
                    Robot testRobot(Config::getInstance().getTestModelName(),ifaceName);
                    testDribbler(testRobot);
                }
                break;
                case testTactics :{
                    testShootTactics();
                    //Robot testRobot(Config::getInstance().getTestModelName(),ifaceName);
                    //testDribbler(testRobot);
                }
                break;
                default:
                break;
            }
    }

    xmlCleanupParser();

    std::cout<<"exit from robocup ssl client bye bye"<<std::endl;
    return 0;
}
#endif
