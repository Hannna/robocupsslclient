/*
 * TestRRT.h
 *
 *  Created on: 2010-11-03
 *      Author: maciek
 */

#ifndef TESTRRT_H_
#define TESTRRT_H_

#include "../Thread/Thread.h"
#include "../Test/Test.h"

class Videoserver;
class Robot;
class goalPose;

class TestRRT: public Thread {
public:
	TestRRT(Robot* robot,Videoserver* video,Pose goalPose);
	virtual ~TestRRT();
private:
    virtual void Execute(void*);
	struct threadArg arg1;
};

#endif /* TESTRRT_H_ */
