/*
 * SimulationException.cpp
 *
 *  Created on: 29-05-2011
 *      Author: maciek
 */

#include "SimulationException.h"

SimulationException::SimulationException(std::string err) {
	this->err = err;

}


const char* SimulationException::what() const throw()
{
	return err.c_str();
}

SimulationException::~SimulationException()throw() {
	// TODO Auto-generated destructor stub
}
