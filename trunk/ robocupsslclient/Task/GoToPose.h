/*
 * GoToPose.h
 *
 *  Created on: 2010-06-07
 *      Author: maciek
 */

#ifndef GOTOPOSE_H_
#define GOTOPOSE_H_

#include "Task.h"
class RRTPlanner;

class GoToPose: public Task {
public:
	GoToPose(const Pose & pose,Robot * robot);
	virtual Task* nextTask();
	virtual ~GoToPose();
protected:
	virtual Task::status run(void * arg, int steps=-1);
private:
	GoToPose();
	GoToPose(const GoToPose &);
	GoToPose& operator=(const GoToPose&);

	RRTPlanner * rrt;
	std::list<Pose>  path;
	const Pose & goalPose;
	bool serialize;

	double currSimTime;
	double lastSimTime;
};

#endif /* GOTOPOSE_H_ */
