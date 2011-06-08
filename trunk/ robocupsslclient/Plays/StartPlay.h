/*
 * StartPlay.h
 *
 *  Created on: 24-05-2011
 *      Author: maciek
 */

#ifndef STARTPLAY_H_
#define STARTPLAY_H_

#include "Play.h"


/*przygotowuje roboty do startu
 * tj. bramkarz cofa sie
 * napastnicy rozstawiaja sie po bokach
 *
 */

class StartPlay : public Play{
public:

	StartPlay(std::string team);

    virtual void execute();
    virtual void reset();
    virtual void stop();
    virtual void waitForFinish( );
	virtual ~StartPlay();
};

#endif /* STARTPLAY_H_ */
