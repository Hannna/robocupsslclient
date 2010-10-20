/*
 * PidRegulator.h
 *
 *  Created on: 2009-11-17
 *      Author: maciek
 */

#ifndef PIDREGULATOR_H_
#define PIDREGULATOR_H_
#include "boost/tuple/tuple.hpp"
#include <math.h>
#include "../Vector2d/Vector2D.h"

class PidRegulator {
public:
	PidRegulator();
	void setCmdVel(Vector2D linVel,double rotVel);
	boost::tuple<double,double,double> calculateNewVel();
	virtual ~PidRegulator();
	double calculateError(Vector2D currentVel,double currRotVel);
private:
	//wzmocnienie
	double K;
	//wspolczynnik rozniczkowania
	double Di;
	//wspolczynnik calkowania
	double Ti;
	Vector2D currLinVel;
	double currRotVel;

};

#endif /* PIDREGULATOR_H_ */
