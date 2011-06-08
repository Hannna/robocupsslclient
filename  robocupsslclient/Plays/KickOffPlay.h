/*
 * KickOffPlay.h
 *
 *  Created on: 24-05-2011
 *      Author: maciek
 */

#ifndef KICKOFFPLAY_H_
#define KICKOFFPLAY_H_

#include "Play.h"

class KickOffPlay: public Play {
public:
	KickOffPlay(std::string teamColor);
	virtual ~KickOffPlay();
};

#endif /* KICKOFFPLAY_H_ */
