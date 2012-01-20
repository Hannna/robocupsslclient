/*
 * FollowLineAndAvoidObs.h
 *
 *  Created on: Oct 2, 2011
 *      Author: maciek
 */

#ifndef FOLLOWLINEANDAVOIDOBS_H_
#define FOLLOWLINEANDAVOIDOBS_H_

#include "Tactic.h"
#include "../Task/GoToPose.h"
/*Taktyka obronna polegajaca na bronieniu wybranej linii
 *
 * Robot porusza sie wzdluz odcinka (p1,p2)  z mozliwoscia odchylanie od niego o maxDistFromLine w kierunku pilki
 *
 */


class FollowLineAndAvoidObs: public Tactic {
public:
	FollowLineAndAvoidObs( Robot & robot_, const Vector2D p1, const Vector2D p2,const std::string iter );
    virtual void execute(void *);
    virtual bool isFinish();
	virtual ~FollowLineAndAvoidObs();

protected:
	const Vector2D p1;
	const Vector2D p2;
	TaskSharedPtr goToPose;
	std::string file_name;
	std::ofstream file;

private:

};

#endif /* FOLLOWLINEANDAVOIDOBS_H_ */
