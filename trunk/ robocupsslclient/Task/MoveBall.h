/*
 * MoveBall.h
 *
 *  Created on: 24-04-2011
 *      Author: maciek
 */

#ifndef MOVEBALL_H_
#define MOVEBALL_H_

#include "Task.h"
#include "GoToPose.h"

class MoveBall: public Task {
public:
	MoveBall(const Pose & pose,Robot * robot);
	virtual Task* nextTask();
	virtual ~MoveBall();
protected:
	virtual Task::status run(void * arg, int steps=-1);
private:
	const Pose goalPose;
	GoToPose * goToPose;
	MoveBall();
	MoveBall(const MoveBall &);
	MoveBall& operator=(const MoveBall&);
};

#endif /* MOVEBALL_H_ */
