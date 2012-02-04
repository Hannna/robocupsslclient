/*
 * SimAnnealing.h
 *
 *  Created on: Jan 15, 2012
 *      Author: maciek
 */

#ifndef SIMANNEALING2_H_
#define SIMANNEALING2_H_

#include "../GameState/GameState.h"
#include "../Robot/Robot.h"
#include "../EvaluationModule/EvaluationModule.h"


class SimAnnealing2 {

public:
	SimAnnealing2( const GameStatePtr& gamestate, const std::string robotName,Robot::robotID rid );
	std::pair<Vector2D, double> simAnnnealing2();
	virtual ~SimAnnealing2();
private:
	const GameStatePtr gameState;
	//const Vector2D goalPose;
	//const Robot::robotID robotID_;
	const std::string robotName;
	Robot::robotID rid;
	const EvaluationModule& evaluation;
};

#endif /* SIMANNEALING_H_ */
