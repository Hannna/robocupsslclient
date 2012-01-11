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
	//jesli odleglosc przekracza ponizsza to konieczna zmiana tasku na GoToBall
	static const double maxDistanceToBall = 0.2;

	GetBall(Robot * robot);
	virtual Task* nextTask();
	virtual ~GetBall();
protected:
	virtual Task::status run(void * arg, int steps=-1);
private:

	GetBall();
	GetBall(const GetBall &);
	GetBall& operator=(const GetBall&);
	std::string file_name;
	std::ofstream file;

};

#endif /* GETBALL_H_ */
