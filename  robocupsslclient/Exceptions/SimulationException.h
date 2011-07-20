/*
 * SimulationException.h
 *
 *  Created on: 29-05-2011
 *      Author: maciek
 */

#ifndef SIMULATIONEXCEPTION_H_
#define SIMULATIONEXCEPTION_H_

// standard exceptions
#include <iostream>
#include <exception>

class SimulationException : public std::exception{
public:
	SimulationException(std::string err);
	virtual const char* what() const throw();
	virtual ~SimulationException()throw();

private:
	std::string err;
};

#endif /* SIMULATIONEXCEPTION_H_ */
