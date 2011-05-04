/*
 * GetBall.h
 *
 *  Created on: 24-04-2011
 *      Author: maciek
 */

#ifndef GETBALL_H_
#define GETBALL_H_
#include "Task.h"

/*@brief getting the ball onto robot dribbler
 *
 */
class GetBall: public Task {
public:
	GetBall(Robot * robot);
	virtual Task* nextTask();
	virtual ~GetBall();
protected:
	virtual Task::status run(void * arg, int steps=-1);
private:

	GetBall();
	GetBall(const GetBall &);
	GetBall& operator=(const GetBall&);
	//jesli odleglosc przekracza ponizsza to konieczna zmiana tasku na GoToBall
	static const double maxDistanceToBall = 0.15;
};

#endif /* GETBALL_H_ */
