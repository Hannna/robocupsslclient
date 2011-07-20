/*
 * ChargeBall.h
 *
 *  Created on: 27-04-2011
 *      Author: maciek
 */

#ifndef CHARGEBALL_H_
#define CHARGEBALL_H_

#include "Tactic.h"

/*Taktyka polegajaca na zdobyciu pilki przez robota
 *
 */
class ChargeBall: public Tactic {
public:
	ChargeBall(Robot & robot_);
	virtual void execute(void*);
	virtual bool isFinish();
	virtual ~ChargeBall();
};

#endif /* CHARGEBALL_H_ */
