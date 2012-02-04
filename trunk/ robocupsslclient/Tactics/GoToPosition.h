/*
 * GoToPosition.h
 *
 *  Created on: Feb 4, 2012
 *      Author: maciek
 */

#ifndef GOTOPOSITION_H_
#define GOTOPOSITION_H_

#include "Tactic.h"

class GoToPosition: public Tactic {
public:
	GoToPosition(Robot & robot_, const Pose & goalPosition);
	virtual void execute(void *);
	virtual bool isFinish();
	virtual ~GoToPosition();
protected:
	const Pose  goalPosition;

};

#endif /* GOTOPOSITION_H_ */
