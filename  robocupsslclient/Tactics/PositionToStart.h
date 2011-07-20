/*
 * PositionToStart.h
 *
 *  Created on: 09-05-2011
 *      Author: maciek
 */

#ifndef POSITIONTOSTART_H_
#define POSITIONTOSTART_H_

#include "Tactic.h"

class PositionToStart: public Tactic {
public:
	PositionToStart( const Pose &startPose, Robot& robot_  );
	PositionToStart( const Pose &startPose, Robot* robot_  );

    virtual bool isFinish();
    virtual ~PositionToStart();
protected:
	 virtual void execute(void*);
	 const Pose startPose;

};

#endif /* POSITIONTOSTART_H_ */
