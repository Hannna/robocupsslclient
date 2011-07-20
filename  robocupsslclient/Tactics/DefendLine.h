/*
 * DefendLine.h
 *
 *  Created on: 02-05-2011
 *      Author: maciek
 */

#ifndef DEFENDLINE_H_
#define DEFENDLINE_H_

#include "Tactic.h"

/*Taktyka obronna polegajaca na bronieniu wybranej linii
 *
 * Robot porusza sie wzdluz odcinka (p1,p2)  z mozliwoscia odchylanie od niego o maxDistFromLine w kierunku pilki
 *
 */

class DefendLine: public Tactic {
public:
	DefendLine(Robot & robot_, const Vector2D p1, const Vector2D p2, const double maxDistFromLine  );
    virtual void execute(void *);
    virtual bool isFinish();
	virtual ~DefendLine();

protected:

	const Vector2D p1;
	const Vector2D p2;
	const double maxDistFromLine;

private:

};

#endif /* DEFENDLINE_H_ */
