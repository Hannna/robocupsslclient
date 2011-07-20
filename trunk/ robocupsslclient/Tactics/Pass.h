/*
 * Pass.h
 *
 *  Created on: 24-04-2011
 *      Author: maciek
 */

#ifndef PASS_H_
#define PASS_H_

#include "../Robot/Robot.h"
#include "Tactic.h"


//taktyka polegajaca na podaniu pilki od innego robota
// z tej samej druzyny
//
//Opis taktyki (SSM -> skill state machine)
//1. sprawdz czy mam pilke, jesli nie to wroc do 1
//2. jesli mam pilke to sprawdz czy mozna podac do robota o danym robotID jesli nie to
//3. wystaw sie na pozycje ( obroc robota )
//4. jesli podanie jest mozliwe to podaj
class Pass: public Tactic {
public:
	Pass( Robot& robot_, const Robot::robotID targetRobotID );

    virtual bool isFinish();
	virtual ~Pass();
protected:
	 virtual void execute(void*) ;
private:
	 Robot::robotID targetRobotID;
};

#endif /* PASS_H_ */
