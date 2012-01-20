/*
 * Experiment1.h
 *
 *  Created on: Oct 2, 2011
 *      Author: maciek
 */

#ifndef EXPERIMENT1_H_
#define EXPERIMENT1_H_

#include "Play.h"

class Experiment_1: public Play {
public:
	Experiment_1( std::string teamColor, const int nrOfROles, const std::string iter_ );
	virtual void execute( );
	virtual void reset( );
	virtual void stop( );
	virtual void waitForFinish( );
	virtual ~Experiment_1( );

private:
	std::string iter;
};

#endif /* EXPERIMENT1_H_ */
