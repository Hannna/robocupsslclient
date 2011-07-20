/*
 * SimplePlay.h
 *
 *  Created on: 08-06-2011
 *      Author: maciek
 */

#ifndef SIMPLEPLAY_H_
#define SIMPLEPLAY_H_

#include "Play.h"

class SimplePlay: public Play {
public:
	SimplePlay( std::string teamColor );
    bool isFinished();
    virtual void execute();
    virtual void reset();
    virtual ~SimplePlay();
protected:

    virtual void waitForFinish( );

    virtual void stop();
};

#endif /* SIMPLEPLAY_H_ */
