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
#include <boost/thread/mutex.hpp>

//class Videoserver;

class Task {
public:
	Task(Robot *robot);
	virtual bool execute();
	virtual void stop();
	virtual ~Task();
protected:
	virtual bool run()=0;
	const Videoserver & video;
	bool stopTask;
	Robot * robot;
	boost::mutex mutex_;
};

#endif /* TASK_H_ */
