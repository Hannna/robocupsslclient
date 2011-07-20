/*
 * Recevive_pass.h
 *
 *  Created on: 24-04-2011
 *      Author: maciek
 */

#ifndef RECEVIVE_PASS_H_
#define RECEVIVE_PASS_H_

#include "Tactic.h"
//taktyka polegajaca na odebraniu pilki od innego robota
// z tej samej druzyny
//
//Opis taktyki
//1. jesli moja druzya nie ma pilki to jedz w kierunku pilki
//2. jesli moja druzyna ma pilke to ustaw sie tak aby byc na wprost robota ktory ma pilke
//3. jesli odebranie podania jest malo prawdopodobne to wyjdz na lepsza pozycje
//4.
class Receive_pass : public Tactic{
public:
	Receive_pass(Robot& robot_);
    virtual void execute(void*) ;
    virtual bool isFinish();
	virtual ~Receive_pass();
};

#endif /* RECEVIVE_PASS_H_ */
