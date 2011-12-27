/*
 * main.cpp
 *
 *  Created on: 2009-05-07
 *      Author: maciek
 */

#include "RefereeClient/RefereeClient.h"
#include "EvaluationModule/EvaluationModule.h"
#include "Test/Test.h"

#include "STP_algorithm/STP.h"

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
#include "Plays/Play.h"

#include "my_signal.h"

#include "TestManager/TestManager.h"
#include "RefereeClient/RefereeClient.h"
#include "STP_algorithm/STP.h"
#include "Test/Experiments.h"



using namespace Tests;

bool run=true;

//extern const std::string ifaceName;

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

    /*
    Vector2D v(0,1);
    Vector2D w(1,0);
    Vector2D ww(0,-1);

    std::cout<<"angle  "<<v<<" to "<<w<<v.angleTo(w) <<std::endl;
    std::cout<<"angle  "<<v<<" to "<<ww<<v.angleTo(ww) <<std::endl;
    return 0;
	*/

//    log4cxx::PropertyConfigurator::configure("log4cxx.properties");
    Config::getInstance().load("/home/maciek/workspace/magisterka/Debug/config.xml");
    Config::getInstance().setTestMode(true);

    Tests::TestKind testKind=Tests::none;
    Experiments::ExperimentKind experimentKind =  Experiments::none;

    if(argc>1){
        if( strncmp(argv[1],"test",4)==0 ){
            Config::getInstance().setTestMode(true);
        }
        else if( strncmp(argv[1],"experiment_1",12)==0 ){
        	experimentKind = Experiments::navigation_2011;
        }
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
            if(strncmp(argv[2],"rotation",3)==0)
                testKind=Tests::rotation;
            if(strncmp(argv[2],"task",4)==0)
                testKind=Tests::taskScheduling;
            if(strncmp(argv[2],"dribbler",8)==0){
                testKind=Tests::ballDribbling;
            }
            if(strncmp(argv[2],"kick",4)==0)
                testKind=Tests::kick;
            if(strncmp(argv[2],"shoot",5)==0)
                testKind=Tests::testShootTactic;
            if(strncmp(argv[2],"pass",4)==0)
                testKind=Tests::testPassTactic;
            if(strncmp(argv[2],"play",4)==0)
                testKind=Tests::play;

            if( testKind==Tests::none ){
            	std::cout<<"unknown  param "<<argv[2]<<std::endl;
            	xmlCleanupParser();
            	exit(0);
            }
        }
    }

    LOG_ERROR(getLoggerPtr ("app_debug"), "logger test " <<"program started with params "<<argv[2]<<" test kind "<<testKind );

    LOG_DEBUG(getLoggerPtr ("red0"), "test");
    LOG_DEBUG(getLoggerPtr ("red1"), "test");
    LOG_DEBUG(getLoggerPtr ("red2"), "test");

    LOG_DEBUG(getLoggerPtr ("blue0"), "test");
    LOG_DEBUG(getLoggerPtr ("blue1"), "test");
    LOG_DEBUG(getLoggerPtr ("blue2"), "test");


    Play::init();

    Videoserver::getInstance().start(NULL);

    //poczekaj az videoserwer pobierze pozycje robotow z symulatora
    GameStatePtr gameState( new GameState() );

	int sleep_status = 0;
    struct timespec req;
    req.tv_sec=0;
    req.tv_nsec=1000000;
    struct timespec rem;
    bzero( &rem, sizeof(rem) );

    while( Videoserver::getInstance().getUpdateDeltaTime() < 0.001 ){
    	nanosleep(&req,&rem);
    }

    while( Videoserver::getInstance().updateGameState( gameState ) < 0.001 ){
    	nanosleep(&req,&rem);
    }

    //RefereeClient::getInstance().start();

    if( experimentKind != Experiments::none ){
    	if( experimentKind == Experiments::navigation_2011 ){
    		//uruchomienie experymentu
    		run_experiment_1();;
    		while(Videoserver::getInstance().updateGameState( gameState ) > 0.001){
    			sleep_status=nanosleep(&req,&rem);
    		};
    	}

    }
    else if(Config::getInstance().isTestMode()){
    	int sleep_status = 0;
        struct timespec req;
        req.tv_sec=0;
        req.tv_nsec=1000000;
        struct timespec rem;
        bzero( &rem, sizeof(rem) );

        //poczekaj az videoserwer pobierze pozycje robotow z symulatora
    /*    GameStatePtr gameState( new GameState() );

        while( Videoserver::getInstance().getUpdateDeltaTime() < 0.001 ){
        	nanosleep(&req,&rem);
        }

        while( Videoserver::getInstance().updateGameState( gameState ) < 0.001 ){
        	nanosleep(&req,&rem);
        }
	*/
        if( testKind == Tests::play ){
        	//uruchom klienta sslbox
        	RefereeClient::getInstance().start();
        	//sleep(1000);
        	//uruchom glowny algorytm sterujacy
        	run_stp();
        	sleep(100);
        }
        else{
			//DO testow algorytmu
			TestManager testManager;
			for(int i=30;i>0;i--){
				LOG_DEBUG(getLoggerPtr ("app_debug"), "adding test");
				testManager.addTest(testKind);
				testManager.startTests();
				while(!testManager.isTestFinished()){
					sleep_status=nanosleep(&req,&rem);
				};
			}
        }
        RefereeClient::getInstance().stop();//killThread();
        RefereeClient::getInstance().join();
        Videoserver::getInstance().killThread();

        //Videoserver::getInstance().join();
    }

    Play::free();
    xmlCleanupParser();

    return 0;
}
#endif
