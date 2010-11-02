/*
 * Task.h
 *
 *  Created on: 2010-06-07
 *      Author: maciek
 */

#ifndef TASK_H_
#define TASK_H_
#include "../additional.h"
#include "../Robot/Robot.h"
#include "../VideoServer/Videoserver.h"

//class Videoserver;

class Task {
public:
	Task(Robot *robot);
	virtual bool execute()=0;
	virtual void stop();
	virtual ~Task();
protected:
	const Videoserver & video;
	bool stopTask;
	Robot * robot;
};

#endif /* TASK_H_ */
