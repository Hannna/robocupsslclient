/*
 * TestManager.h
 *
 *  Created on: 06-04-2011
 *      Author: maciek
 */

#ifndef TESTMANAGER_H_
#define TESTMANAGER_H_

#include <pthread.h>
#include <deque>
#include <boost/thread/mutex.hpp>

#include "../Logger/Logger.h"
#include "../Thread/Thread.h"

namespace Tests{
 typedef   enum TestKind_{
            none = -1,
            position = 0,
            velocity = 1,
            RRTPlanner = 2,
            multiRRTPlanner = 3,
            acceleration = 4,
            motion = 5,
            rotation = 6,
            taskScheduling = 7,
            ballDribbling = 8,
            kick =9,
            testShootTactic = 10,
            testPassTactic = 11,
            refereeBox = 12,
            play = 13

            } TestKind;

            typedef void * (*funcPtr)(void *) ;
}


class TestManager {
public:
	TestManager();
	void startTests();
	void reset();
	void abort();
	void addTest(Tests::TestKind testKind);
	bool isTestFinished();
	virtual ~TestManager();

private:
	void addTest(Tests::funcPtr t, void* arg=NULL);
//	void addTest(Thread* t);

	typedef struct testFuncArg_{
		Tests::funcPtr testFunc;
		void* arg;
		boost::mutex* finishedTest;
	} testFuncArg;

	std::deque<std::pair<Tests::funcPtr,void*> > tests_fun;
	std::deque<Thread*> tests_th;
	pthread_t currentTestTid;
	const log4cxx::LoggerPtr logger;

	static void* testFunction(void * args);
	boost::mutex finishedTest;

};

#endif /* TESTMANAGER_H_ */