/*
 * ChargeBall.cpp
 *
 *  Created on: 27-04-2011
 *      Author: maciek
 */

#include "ChargeBall.h"

ChargeBall::ChargeBall(Robot & robot_):Tactic(robot_) {


}

void ChargeBall::execute(void*){

	//jesli pilka jest wolna jedz do pilki
	//w p.p trzeba zmienic taktyke (implementacja naiwna)



}

bool ChargeBall::isFinish(){

	return false;
}

ChargeBall::~ChargeBall() {
	// TODO Auto-generated destructor stub
}
