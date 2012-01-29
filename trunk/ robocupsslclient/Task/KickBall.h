/*
 * KickBall.h
 *
 *  Created on: 2010-06-07
 *      Author: maciek
 */

#ifndef KICKBALL_H_
#define KICKBALL_H_

#include "Task.h"

class Robot;

class KickBall : public Task{
public:
    /*@brief kopnij pilke w zadanym kierunku
    *
    */
	KickBall( Robot * robot, const Pose targetPose );
	//KickBall( Robot * robot );
	Task* nextTask();
	//TaskSharedPtr& nextTask();
	virtual ~KickBall();
protected:
	//czy strzal ma zostac oddany natychmiast
	const bool kickNow;

	// rotacja robota ktora ma osiagnac, podczas oddawania strzalu
	const double rotation;
	virtual Task::status run(void*, int steps=-1);
	double calculateAngularVel2(const Pose & currRobotPose, const double goalRotation);

	const Pose targetPose;


};

#endif /* KICKBALL_H_ */
