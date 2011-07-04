/*
 * NaivePlay.h
 *
 *  Created on: 08-05-2011
 *      Author: maciek
 */

#ifndef SIMPLEPLAY_H_
#define SIMPLEPLAY_H_

#include "../Plays/Play.h"
#include "../Robot/Robot.h"

class Tactic;

/*Plan gry oparty na prostym schemacie,
 *
 * kazdy z robotow probuje zdobyc pilke,
 * gdy ja zdobedzie podaza w kierunku bramki i oddaje strzal
 *
 */

class NaivePlay: public Play {
public:
	NaivePlay( std::string teamColor );
    bool isFinished();
    virtual void execute();
    virtual void reset();

	virtual ~NaivePlay();
protected:
    /*przygotowuje roboty do startu
     * tj. bramkarz cofa sie
     * napastnicy rozstawiaja sie po bokach
     *
     */
   // virtual void prepareForStart();

    virtual void waitForFinish( );
    /*przygotowuje roboty do wykopu pilki ze srodka boiska
     *
     */
//    virtual void prepareForKickOff( const Vector2D& kickoffPose );
    /*wszystkie roboty zatrzymuja sie na boisku
     *
     */
    //virtual void halt();
    /*wszystkie roboty zatrzymuja sie co najmniej 30 cm od pilki
     *
     */
    virtual void stop();




//	std::list<AbstractTactic *> tactics;
};

#endif /* SIMPLEPLAY_H_ */
