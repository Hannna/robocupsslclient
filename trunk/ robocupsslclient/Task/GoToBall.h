/*
 * GoToBall.h
 *
 *  Created on: 25-04-2011
 *      Author: maciek
 */

#ifndef GOTOBALL_H_
#define GOTOBALL_H_

#include "Task.h"
#include "GoToPose.h"


class GoToBall: public Task {
public:
	GoToBall(Robot * robot);
	virtual Task* nextTask();
	virtual ~GoToBall();
protected:
	virtual Task::status run(void * arg, int steps=-1);
private:
	GoToPose* goToPose;
	GoToBall();
	GoToBall(const GoToBall &);
	GoToBall& operator=(const GoToBall&);
	Pose ballPose;
	//static const double distToChangeTask = 0.1;//10 cm
};

#endif /* GOTOBALL_H_ */
