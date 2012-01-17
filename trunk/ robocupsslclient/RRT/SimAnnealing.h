/*
 * SimAnnealing.h
 *
 *  Created on: Jan 15, 2012
 *      Author: maciek
 */

#ifndef SIMANNEALING_H_
#define SIMANNEALING_H_

#include "../GameState/GameState.h"
#include "../GameState/GameState.h"
#include "../Robot/Robot.h"

class SimAnnealing {
public:
	SimAnnealing(const GameStatePtr& gamestate, const Vector2D goalPose, Robot::robotID robotID_);
	Vector2D simAnnnealing();
	virtual ~SimAnnealing();
private:
	const GameStatePtr gameState;
	const Vector2D goalPose;
	const Robot::robotID robotID_;
};

#endif /* SIMANNEALING_H_ */
