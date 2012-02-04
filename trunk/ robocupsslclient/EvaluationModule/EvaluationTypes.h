/*
 * EvaluationTypes.h
 *
 *  Created on: Feb 3, 2012
 *      Author: maciek
 */

#ifndef EVALUATIONTYPES_H_
#define EVALUATIONTYPES_H_
namespace BallState{
	enum ballState{
		//pilka jest wolna
		free,
		//pilka na aucie
		out,
		//pilka w bramce
		in_goal,
		//pilka zajmowana przez naszych
		occupied_our,
		//pilka zajmowana przez przeciwnika
		occupied_theirs,
		//
		mine,
		go_to_goal
	};
}

std::ostream & operator<<(std::ostream & os, const BallState::ballState & bState );

#endif /* EVALUATIONTYPES_H_ */
