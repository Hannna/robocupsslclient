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
	KickBall( Robot * robot, const Vector2D targetPose, const double rotation , double maxForce );
	//KickBall( Robot * robot );
	Task* nextTask();
	//TaskSharedPtr& nextTask();
	virtual ~KickBall();
protected:
	//czy strzal ma zostac oddany natychmiast
	const bool kickNow;
	// rotacja robota ktora ma osiagnac, podczas oddawania strzalu
	const double rotation;
	//pozycja w kierunku ktorej mamy oddac strzal
	const Vector2D targetPosition;

	const double maxKickForce;

	virtual Task::status run(void*, int steps=-1);
	double calculateAngularVel2(const Pose & currRobotPose, const double goalRotation);



};

#endif /* KICKBALL_H_ */
