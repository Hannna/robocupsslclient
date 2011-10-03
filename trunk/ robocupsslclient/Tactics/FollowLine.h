/*
 * FollowLine.h
 *
 *  Created on: Oct 2, 2011
 *      Author: maciek
 */

#ifndef FOLLOWLINE_H_
#define FOLLOWLINE_H_

#include "Tactic.h"

/*Taktyka polegajaca na poruszaniu sie wzdluz wybranej linii
 *
 * Robot porusza sie wzdluz odcinka (p1,p2)
 *
 */
class FollowLine: public Tactic {
public:
	FollowLine( Robot & robot_, const Vector2D p1, const Vector2D p2  );
	virtual void execute(void *);
	virtual bool isFinish();
	virtual ~FollowLine();
protected:
	const Vector2D p1;
	const Vector2D p2;
};

#endif /* FOLLOWLINE_H_ */
