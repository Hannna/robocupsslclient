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
#include <fstream>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>

#include "log4cxx/propertyconfigurator.h"

#include "Logger/Logger.h"
#include "Config/Config.h"
#include "my_signal.h"

#include "TestManager/TestManager.h"
#include "RefereeClient/RefereeClient.h"

using namespace Tests;

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


using namespace Tests;

int main(int argc, char*argv[],char *envp[]){

    //daemonInit ( );
    sys_catch_signals();
    // read config files
    //Init libxml
    xmlInitParser();

    log4cxx::PropertyConfigurator::configure("log4cxx.properties");
    Config::getInstance().load("/home/maciek/codeblocks/magisterka/bin/Debug/config.xml");
    Config::getInstance().setTestMode(true);

    Tests::TestKind testKind=Tests::none;

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
            if(strncmp(argv[2],"kick",4)==0)
                testKind=Tests::kick;
            if(strncmp(argv[2],"tactic",6)==0)
                testKind=Tests::testTactics;
        }
        else{
            std::cout<<"missing param"<<std::endl;
            exit(0);
        }
    }

    int sleep_status;
    if(Config::getInstance().isTestMode()){
        std::cout<<"starting test mode"<<std::endl;
        //RefereeClient referee;
        //referee.start();

        //while(referee.getCommand()!=RefereeCommands::start){
        //	usleep(1000);
        //};

        Videoserver::getInstance().start(NULL);

        TestManager testManager;
        struct timespec req;
        req.tv_sec=0;
        req.tv_nsec=1000000;
        struct timespec rem;
        bzero( &rem, sizeof(rem) );

        for(int i=30;i>0;i--){
			testManager.addTest(testKind);
			testManager.startTests();
			while(!testManager.isTestFinished()){
				sleep_status=nanosleep(&req,&rem);
			};
        }

        Videoserver::getInstance().killThread();
    }

    xmlCleanupParser();

    std::cout<<"exit from robocup ssl client bye bye sleep_status"<<sleep_status<<std::endl;
    return 0;
}
#endif
