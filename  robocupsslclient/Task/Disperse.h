/*
 * Disperse.h
 *
 *  Created on: Aug 9, 2011
 *      Author: maciek
 */

#ifndef DISPERSE_H_
#define DISPERSE_H_

#include "Task.h"

class Disperse: public Task {
public:
	//jesli odleglosc przekracza ponizsza to konieczna zmiana tasku na GoToBall
	static const double minimalDistanceToObstacle = 0.5; //m
	static const double maxDisperseVelocity = 0.5; //m/s

	Disperse(Robot * robot);
	virtual Task* nextTask();

protected:
	virtual Task::status run(void * arg, int steps=-1);
private:

	Disperse();
	Disperse(const Disperse &);
	Disperse& operator=(const Disperse&);

	virtual ~Disperse();
};



#endif /* DISPERSE_H_ */
