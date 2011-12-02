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
	GoToPose(const Pose & pose,Robot * robot, double maxDistToGoal = Config::getInstance().getRRTMinDistance() );
	GoToPose(const Pose & pose,Robot * robot, bool force, double maxDistToGoal = Config::getInstance().getRRTMinDistance());

	void addXConstraint(  std::pair<double, double> *xConstraints ){
		this->xConstraints = xConstraints;
		//LOG_INFO( log,"dodano ograniczenia x:["<<xConstraints->first <<";"<< xConstraints->second<<"]" );
	}

	void addYConstraint( std::pair<double, double> *yConstraints ){
		this->yConstraints = yConstraints;
		//LOG_INFO( log,"dodano ograniczenia y:["<<yConstraints->first <<";"<< yConstraints->second<<"]" );
	}

	virtual Task* nextTask();
	virtual ~GoToPose();
protected:
	virtual Task::status run(void * arg, int steps=-1);
private:
	GoToPose();
	GoToPose(const GoToPose &);
	GoToPose& operator=(const GoToPose&);

	std::pair<double, double> *xConstraints;
	std::pair<double, double> *yConstraints;

	RRTPlanner * rrt;
	std::list<Pose>  path;
	const Pose goalPose;
	bool serialize;

	const double maxDistToGoal;
	double currSimTime;
	double lastSimTime;
	bool force;
};

#endif /* GOTOPOSE_H_ */
