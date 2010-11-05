/*
 * GoToPose.h
 *
 *  Created on: 2010-06-07
 *      Author: maciek
 */

#ifndef GOTOPOSE_H_
#define GOTOPOSE_H_

#include "Task.h"

class GoToPose: public Task {
public:
	GoToPose(const Pose & pose,Robot * robot);
	bool execute();
	virtual ~GoToPose();
private:
	const Pose & goalPose;

};

#endif /* GOTOPOSE_H_ */