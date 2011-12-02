/*
 * RoundObstacle.h
 *
 *  Created on: Oct 3, 2011
 *      Author: maciek
 */

#ifndef ROUNDOBSTACLE_H_
#define ROUNDOBSTACLE_H_

#include "Task.h"
#include "GoToPose.h"

/*zadanie polegajace na okrazeniu przeszkody
 *
 * Przebieg.
 *
 *
 *
 */
class RoundObstacle: public Task {
public:
	enum direction{
		LEFT,
		RIGHT
	};

	RoundObstacle(Robot * robot, const Vector2D obstacleCoordinates, enum direction d );
	virtual Task* nextTask();
	virtual ~RoundObstacle();
protected:
	virtual Task::status run(void * arg, int steps=-1);
private:
	RoundObstacle();
	RoundObstacle(const RoundObstacle & r);
	RoundObstacle& operator=(const RoundObstacle& r);

	void initBezierParams(const Vector2D a,const  Vector2D b,const  Vector2D c,const  Vector2D d);
	/*@brief oblicza wspolrzedne punktu lezacego na krzywej beziera t nalezy do (0,1)
	 *
	 */
	Vector2D bezierCurve(double n,Vector2D bezierParams[], double t);

	Vector2D bezierParams[5];

	const Vector2D obstacleCoordinates;
	GoToPose* goToPose;
	double t;
	enum direction d;
};



#endif /* ROUNDOBSTACLE_H_ */
