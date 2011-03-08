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
	KickBall(Robot * robot, double rotation);
	virtual ~KickBall();
protected:
    const double rotation;
	virtual bool run(void*, int steps=-1);
};

#endif /* KICKBALL_H_ */
