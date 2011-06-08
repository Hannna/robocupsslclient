#ifndef ABSTRACTPLAN_H
#define ABSTRACTPLAN_H

#include "../Logger/Logger.h"
#include "../Role/Role.h"

/*
 * simple example of play
PLAY Naive Offense
APPLICABLE offense
DONE aborted !offense
ROLE 1
shoot A
none
ROLE 2
defend_point {-1400 250} 0 700
none
ROLE 3
defend_lane {B 0 -200} {B 1175 -200}
none
ROLE 4
defend_
*/

#include "../Config/Config.h"

//class Config;
class Robot;
class Vector2D;
class Role;


class Play
{
    public:
        Play(std::string teamColor);
        bool isFinished();
        virtual void execute()=0;
        virtual void reset()=0;
        virtual ~Play();

        /*przygotowuje roboty do startu
         * tj. bramkarz cofa sie
         * napastnicy rozstawiaja sie po bokach
         *
         */
        //virtual void prepareForStart()=0;

        virtual void waitForFinish( )=0;
        /*przygotowuje roboty do wykopu pilki ze srodka boiska
         *
         */
//        virtual void prepareForKickOff(const Vector2D& kickoffPose)=0;
        /*wszystkie roboty zatrzymuja sie na boisku
         *
         */
        virtual void halt();
        /*wszystkie roboty zatrzymuja sie co najmniej 30 cm od pilki
         *
         */
        virtual void stop()=0;

        Config & appConfig;
        static void init();
        static void free();
    protected:
    const log4cxx::LoggerPtr log;

    std::string teamColor;

    Role role0;
	Role role1;
	Role role2;

//	Robot* robot0;
//	Robot* robot1;
//	Robot* robot2;

	static Robot* red0;
	static Robot* red1;
	static Robot* red2;

	static Robot* blue0;
	static Robot* blue1;
	static Robot* blue2;

};

#endif // ABSTRACTPLAN_H
