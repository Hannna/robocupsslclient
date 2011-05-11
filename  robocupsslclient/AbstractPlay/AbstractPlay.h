#ifndef ABSTRACTPLAN_H
#define ABSTRACTPLAN_H

#include "../Logger/Logger.h"

class Config;
class Robot;

class AbstractPlay
{
    public:
        AbstractPlay();
        bool isFinished();
        virtual void execute()=0;
        virtual void reset()=0;
        virtual ~AbstractPlay();

        /*przygotowuje roboty do startu
         * tj. bramkarz cofa sie
         * napastnicy rozstawiaja sie po bokach
         *
         */
        virtual void prepareForStart()=0;
        /*przygotowuje roboty do wykopu pilki ze srodka boiska
         *
         */
        virtual void prepareForKickOff()=0;
        /*wszystkie roboty zatrzymuja sie na boisku
         *
         */
        virtual void halt();
        /*wszystkie roboty zatrzymuja sie co najmniej 30 cm od pilki
         *
         */
        virtual void stop()=0;

        //virtual void play()=0;

        virtual void freeKick()=0;

        Config & appConfig;

    protected:
    const log4cxx::LoggerPtr log;

	Robot* red0;
	Robot* red1;
	Robot* red2;
	Robot* blue0;
	Robot* blue1;
	Robot* blue2;
};

#endif // ABSTRACTPLAN_H
