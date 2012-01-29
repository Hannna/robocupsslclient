/*
 * PositionToPass.h
 *
 *  Created on: Jan 28, 2012
 *      Author: maciek
 */

#ifndef POSITIONTOPASS_H_
#define POSITIONTOPASS_H_

#include "Tactic.h"

class PositionToPass: public Tactic{
public:
	PositionToPass(Robot& robot_);
    virtual void execute(void*) ;
    virtual bool isFinish();
	virtual ~PositionToPass();
};

#endif /* POSITIONTOPASS_H_ */
