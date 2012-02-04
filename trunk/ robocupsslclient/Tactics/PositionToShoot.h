/*
 * PositionToShoot.h
 *
 *  Created on: Feb 3, 2012
 *      Author: maciek
 */

#ifndef POSITIONTOSHOOT_H_
#define POSITIONTOSHOOT_H_

#include "Tactic.h"

class PositionToShoot: public Tactic {
public:
	PositionToShoot(Robot& robot_);
	virtual void execute(void*) ;
	virtual bool isFinish();
	virtual ~PositionToShoot();
};

#endif /* POSITIONTOSHOOT_H_ */
