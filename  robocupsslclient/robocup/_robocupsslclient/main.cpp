/*
 * main.cpp
 *
 *  Created on: 2009-05-07
 *      Author: maciek
 */

#include "RefereeClient/RefereeClient.h"
#include "Test/Test.h"

#ifdef TEST
    #define BOOST_TEST_DYN_LINK
    //#define BOOST_TEST_MAIN
    //#define BOOST_TEST_MODULE hhh

    #include <boost/test/unit_test.hpp>
    using namespace boost::unit_test_framework;
    #include <iostream>

    bool init_unit_test(){
        boost::unit_test::test_suite* ts1 = BOOST_TEST_SUITE( "test_suite1" );

        boost::shared_ptr<RefereeClient> instance( new RefereeClient( ) );
        framework::master_test_suite().add( BOOST_CLASS_TEST_CASE( &RefereeClient::testConnection, instance ) );
        ts1->add( BOOST_CLASS_TEST_CASE( &RefereeClient::testConnection, instance ) );

        boost::unit_test::framework::master_test_suite().add(ts1);
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


#include "Logger/Logger.h"
#include "Config/Config.h"
#include "Robot/Robot.h"
#include "signal.h"

bool run=true;

extern const std::string ifaceName;

int main(int argc, char*argv[],char *envp[]){

		sys_catch_signals();
		// read config files
		//Init libxml
		xmlInitParser();
		bool testPosition=false;
		bool testVelocity=false;
		bool testRRTPlanner=false;
		bool testAcc=false;
		bool testMotion_=false;
		bool testRot=false;
		bool testTask=true;
		Logger::getInstance().createLoggers();
		Logger::getInstance().disableCout();//Logger::getInstance().enableCout();

		Config::getInstance().load("/home/maciek/codeblocks/magisterka/bin/Debug/config.xml");
		Config::getInstance().setTestMode(true);
		if(argc>1){
			if(strncmp(argv[1],"test",4)==0)
				Config::getInstance().setTestMode(true);
			if(strncmp(argv[2],"velocity",8)==0)
				testVelocity=true;
			if(strncmp(argv[2],"position",8)==0)
				testPosition=true;
			if(strncmp(argv[2],"rrt",3)==0)
				testRRTPlanner=true;
			if(strncmp(argv[2],"acc",3)==0)
				testAcc=true;
			if(strncmp(argv[2],"motion",3)==0)
				testMotion_=true;
			if(strncmp(argv[2],"rot",3)==0)
				testRot=true;
			if(strncmp(argv[2],"task",4)==0)
				testTask=true;
		}

		if(Config::getInstance().isTestMode()){
			std::cout<<"starting test mode"<<std::endl;

			if(testVelocity){
				Robot testRobot(Config::getInstance().getTestModelName(),ifaceName);
				std::vector<boost::tuple<double,double,double> > tests=Config::getInstance().getVelTests();
				std::vector<boost::tuple<double,double,double> >::const_iterator ii=tests.begin();
				for(;ii!=tests.end();ii++){
					Vector2D vel(ii->get<0>(),ii->get<1>());
					testVel(vel,ii->get<2>(),testRobot,Config::getInstance().getTestEstimatedTime());
				}
			}
			else if(testPosition){
				Robot testRobot(Config::getInstance().getTestModelName(),ifaceName);
				std::cout<<"starting position tests"<<std::endl;
				std::vector<Pose> tests=Config::getInstance().getPoseTests();
				std::vector<Pose>::const_iterator ii = tests.begin();
				for(;ii!=tests.end();ii++){
					Vector2D vel(ii->get<0>(),ii->get<1>());
					testPose(testRobot,*ii);
				}
			}
			else if(testAcc){
				Robot testRobot(Config::getInstance().getTestModelName(),ifaceName);
				Vector2D speed(1,1);
				std::cout<<"starting acceleration tests"<<std::endl;
				int i=30;
				while(i--)
					checkAcceleration(speed,Videoserver::getInstance(),testRobot);
			}
			else if(testMotion_){
				Robot testRobot(Config::getInstance().getTestModelName(),ifaceName);
				std::cout<<"starting motion tests"<<std::endl;

				testMotion(Config::getInstance().getRRTGoalPose(),Videoserver::getInstance(),testRobot);
				testMotion(Pose (1,0,0),Videoserver::getInstance(),testRobot);
				testMotion(Pose (1,1,0),Videoserver::getInstance(),testRobot);
				testMotion(Pose (-1,-1,0),Videoserver::getInstance(),testRobot);
			}
			else if(testRot){
				Robot testRobot(Config::getInstance().getTestModelName(),ifaceName);
				std::cout<<"starting rotation tests"<<std::endl;
				testRotation(Videoserver::getInstance(),testRobot);
			}
            else if(testRRTPlanner){
				std::cout<<"starting RRT tests"<<std::endl;
				testRRTThread(Videoserver::getInstance());
				//testRRT(Config::getInstance().getRRTGoalPose(),&videoserver,&testRobot);
			}
			else if(testTask){
				std::cout<<"starting tasks tests"<<std::endl;
				testTaskThread();
			}
		}

		xmlCleanupParser();

		std::cout<<"bye bye"<<std::endl;
		return 0;
}
#endif
