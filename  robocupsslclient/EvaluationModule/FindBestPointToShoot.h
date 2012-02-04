/*
 * FindBestPointToShoot.h
 *
 *  Created on: Feb 4, 2012
 *      Author: maciek
 */

#ifndef FINDBESTPOINTTOSHOOT_H_
#define FINDBESTPOINTTOSHOOT_H_
#include "../Vector2d/Vector2D.h"
#include "../GameState/GameState.h"
#include "../EvaluationModule/EvaluationModule.h"
#include <boost/random.hpp>

class FindBestPointToShoot {
public:
	//znajduje najlepszy pkt do strzalu w otoczeniu o zasiegu distance [mm] punktu point
	FindBestPointToShoot(const Vector2D point, const int distance,	const GameStatePtr& gamestate_,  const std::string robotName_,Robot::robotID rid_);
	std::pair<Vector2D, double> bestTarget();
	virtual ~FindBestPointToShoot();
private:
	static boost::mt19937 rngX;
	//generator wspolrzednej Y
	static boost::mt19937 rngY;

	const Vector2D point;
	const int distance;
	const GameStatePtr& gamestate;
	const std::string robotName;
	const Robot::robotID rid;
	const EvaluationModule& evaluation;
};

#endif /* FINDBESTPOINTTOSHOOT_H_ */
