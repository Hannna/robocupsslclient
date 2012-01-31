/*
 * DribbleToShoot.h
 *
 *  Created on: Jan 29, 2012
 *      Author: maciek
 */

#ifndef DRIBBLETOSHOOT_H_
#define DRIBBLETOSHOOT_H_
#include "Tactic.h"

class DribbleToShoot: public Tactic {
public:
	DribbleToShoot(Robot& robot_);
    virtual void execute(void*) ;
    virtual bool isFinish();
	virtual ~DribbleToShoot();
};

#endif /* DRIBBLETOSHOOT_H_ */
