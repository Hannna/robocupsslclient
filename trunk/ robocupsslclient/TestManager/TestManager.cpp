/*
 * TestManager.cpp
 *
 *  Created on: 06-04-2011
 *      Author: maciek
 */

#include "TestManager.h"

#include "../Config/Config.h"
#include "../Test/Test.h"
#include "../RefereeClient/RefereeClient.h"

using namespace Tests;

TestManager::TestManager():logger(getLoggerPtr("app_debug")) {
	this->finishedTest.try_lock();
}

void TestManager::startTests(){

	pthread_attr_t  attr;
	pthread_attr_init(&attr);
	static testFuncArg arg;
	arg.testFunc = this->tests_fun.front().first;
	arg.arg = this->tests_fun.front().second;
	this->tests_fun.pop_front();
	this->finishedTest.unlock();

	arg.finishedTest = &this->finishedTest;

	finishedTest.lock();
	pthread_create(&currentTestTid, &attr, testFunction, &arg);

}

void TestManager::addTest(Tests::funcPtr t, void* arg){
	std::pair<Tests::funcPtr,void* > p(t,arg);
	this->tests_fun.push_back( p );

}

void TestManager::addTest(Thread* t){
	this->tests_th.push_back(t);

}

void TestManager::addTest(Tests::TestKind testKind){
	//this->tests_fun.push_back(t);
	funcPtr test;
    switch(testKind){
    	case velocity: {
    		//LOG_DEBUG(logger,"not supported test");
            Robot testRobot(Config::getInstance().getTestModelName(),ifaceName);
            std::vector<boost::tuple<double,double,double> > tests=Config::getInstance().getVelTests();
            std::vector<boost::tuple<double,double,double> >::const_iterator ii=tests.begin();
            for(;ii!=tests.end();ii++){
            	Vector2D vel(ii->get<0>(),ii->get<1>());
                sleep(1);
                testVel(vel,ii->get<2>(),testRobot,Config::getInstance().getTestEstimatedTime());
            }

    	}
        break;
        case position: {
        	//LOG_DEBUG(logger,"not supported test");
        	Robot testRobot(Config::getInstance().getTestModelName(),ifaceName);
            std::vector<Pose> tests=Config::getInstance().getPoseTests();
            std::vector<Pose>::const_iterator ii = tests.begin();
            for(;ii!=tests.end();ii++){
            	Vector2D vel(ii->get<0>(),ii->get<1>());
                testPose(testRobot,*ii);
            }

       }
       break;
       case acceleration: {
    	   LOG_INFO(logger," starting acceleration test ");
    	   Robot testRobot(Config::getInstance().getTestModelName(),ifaceName);
           /*
           Robot redRobot1(std::string("red1"),ifaceName);
           Robot redRobot2(std::string("red2"),ifaceName);

           Robot blueRobot0(std::string("blue0"),ifaceName);
           Robot blueRobot1(std::string("blue1"),ifaceName);
           Robot blueRobot2(std::string("blue2"),ifaceName);
            */
           Videoserver::getInstance().start(NULL);

           std::vector<boost::tuple<double,double,double> > tests=Config::getInstance().getVelTests();
           std::vector<boost::tuple<double,double,double> >::const_iterator ii=tests.begin();
           for(;ii!=tests.end();ii++){
        	   Vector2D speed(ii->get<0>(),ii->get<1>());
        	   speed = speed*Config::getInstance().getSpeedFactor();
               checkAcceleration(speed,testRobot);

               sleep(1);
               SimControl::getInstance().restart();
               sleep(1);
           }

        }
		break;
		case motion: {
			LOG_DEBUG(logger,"not supported test");
			Robot testRobot(Config::getInstance().getTestModelName(),ifaceName);

			//testMotion(Config::getInstance().getRRTGoalPose(),Videoserver::getInstance(),testRobot);
			Pose tmp(0.5,0.5,0);
			testMotion( Config::getInstance().field.FIELD_MIDDLE_POSE + tmp ,Videoserver::getInstance(),testRobot);
			testMotion( Config::getInstance().field.FIELD_MIDDLE_POSE - tmp,Videoserver::getInstance(),testRobot);
			tmp = Pose(0.5,0,0);
			testMotion( Config::getInstance().field.FIELD_MIDDLE_POSE + tmp ,Videoserver::getInstance(),testRobot);
			testMotion( Config::getInstance().field.FIELD_MIDDLE_POSE - tmp,Videoserver::getInstance(),testRobot);
			tmp = Pose(0,0.5,0);
			testMotion( Config::getInstance().field.FIELD_MIDDLE_POSE + tmp ,Videoserver::getInstance(),testRobot);
			testMotion( Config::getInstance().field.FIELD_MIDDLE_POSE - tmp,Videoserver::getInstance(),testRobot);

		}
		break;
		case rotation: {
			LOG_DEBUG(logger,"not supported test");
			//Robot testRobot(Config::getInstance().getTestModelName(),ifaceName);
			//testRotation(Videoserver::getInstance(),testRobot);
		}
		break;
		case RRTPlanner: {
			test=reinterpret_cast<funcPtr>(&testSingleRRTThread);
			this->tests_fun.push_back( std::pair<funcPtr,void*>(test,NULL) );
		}
		break;
		case multiRRTPlanner: {
			test=reinterpret_cast<funcPtr>(&testMultiRRTThread);
			this->tests_fun.push_back( std::pair<funcPtr,void*>(test,NULL) );
		}
		break;
		case  taskScheduling: {
			test= reinterpret_cast<funcPtr>(&testTaskThread);
			this->tests_fun.push_back( std::pair<funcPtr,void*>(test,NULL) );
		}
		break;
		case ballDribbling: {
			LOG_DEBUG(logger,"not supported test");
			//Robot testRobot(Config::getInstance().getTestModelName(),ifaceName);
			//testDribbler(testRobot);
		}
		break;
		case kick: {
			test=reinterpret_cast<funcPtr>(&testKick);
			this->tests_fun.push_back( std::pair<funcPtr,void*>(test,NULL) );
		}
		break;
		case testTactics:{
			static Robot red0(std::string("red0"),ifaceName);
			test=reinterpret_cast<funcPtr>(&testShootTactics);
			this->tests_fun.push_back( std::pair<funcPtr,void*>(test,reinterpret_cast<void*>(&red0) ) );
		}
		break;
		case refereeBox :{
			LOG_DEBUG(logger,"not supported test");
			//RefereeClient refereeClient;
			//refereeClient.start();
			//refereeClient.join();
		}
		break;
		default:
		break;
	}
}

bool TestManager::isTestFinished(){
	return this->finishedTest.try_lock();

}

void* TestManager::testFunction(void * args){
	testFuncArg * testArgs = reinterpret_cast< testFuncArg * >( args );
	(*testArgs->testFunc)(testArgs->arg);
	testArgs->finishedTest->unlock();

	return NULL;
}

TestManager::~TestManager() {
	// TODO Auto-generated destructor stub
}
