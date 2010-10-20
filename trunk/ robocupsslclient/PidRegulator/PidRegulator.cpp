/*
 * PidRegulator.cpp
 *
 *  Created on: 2009-11-17
 *      Author: maciek
 */

#include "PidRegulator.h"

PidRegulator::PidRegulator() {
	// TODO Auto-generated constructor stub

}
void PidRegulator::setCmdVel(Vector2D linVel,double rotVel){
	this->currLinVel=linVel;
	this->currRotVel=rotVel;
}
double PidRegulator::calculateError(Vector2D currLinVel,double currRotVel){
	//obliczam uchyb regulatora
	return ( pow((this->currRotVel-currRotVel),2)+pow((this->currLinVel.x-currLinVel.x),2) +
			pow((this->currLinVel.y-currLinVel.y),2) );
}
boost::tuple<double,double,double> PidRegulator::calculateNewVel(){


	return boost::tuple<double,double,double>(0,0,0);
}
PidRegulator::~PidRegulator() {
	// TODO Auto-generated destructor stub
}
