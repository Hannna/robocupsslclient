/*
 * Test1Play.h
 *
 *  Created on: Sep 29, 2011
 *      Author: maciek
 */

#ifndef ObstaclesPlay_H_
#define ObstaclesPlay_H_

#include "Play.h"

class ObstaclesPlay: public Play {
public:
	ObstaclesPlay(std::string teamColor, const int nrOfROles);
	virtual void execute();
	virtual void reset();
	virtual void stop();
	virtual void waitForFinish( );
	virtual bool isFinished();
	virtual ~ObstaclesPlay();
};

#endif /* ObstaclesPlay_H_ */
