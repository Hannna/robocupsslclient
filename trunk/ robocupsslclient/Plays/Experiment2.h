/*
 * Experiment2.h
 *
 *  Created on: Jan 28, 2012
 *      Author: maciek
 */

#ifndef EXPERIMENT2_H_
#define EXPERIMENT2_H_

#include "Play.h"

class Experiment_2: public Play {
public:
	Experiment_2( std::string teamColor, const int nrOfROles );
	virtual void execute( );
	virtual void reset( );
	virtual void stop( );
	virtual void waitForFinish( );
	virtual bool isFinished( );
	virtual void updateState(bool forceChangeTactics );
	virtual ~Experiment_2( );
private:
	double startSimTime;
};
#endif /* EXPERIMENT2_H_ */
