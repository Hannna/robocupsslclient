#ifndef ABSTRACTTACTIC_H
#define ABSTRACTTACTIC_H

/*
@brief klasa reprezentujaca plan dzialania dla pojedyczego robota

Przez Tactics rozumiany jest plan działań dla jednej roli. Można zatem to rozumieć jako
plan działań na szczeblu robota prowadzący do osiągnięcia efektu. Przykładem może być
strzał na bramkę. Robot dostaje polecenie oddania strzału na bramkę, zatem plan jego
poczynań ma doprowadzić do sytuacji, w której osiągnie on pozycję umożliwiającą strzał na
bramkę z zadanym powodzeniem. Plan na szczeblu pojedynczego robota jest wykonywany
do momentu zmiany planu gry calej druzyny. Przykładowe plany działań dla robotów:
   • strzał na bramkę,
   • podanie piłki,
   • odebranie podania,
   • blokowanie innego robota,
   • wyjście na pozycję,
   • bronienie pozycji,
   • dryblowanie z piłką.

*/

#include "../Task/Task.h"
#include "../Robot/Robot.h"
#include "../Thread/Thread.h"
class EvaluationModule;

class AbstractTactic : public Thread
{
    public:
        AbstractTactic(Robot & robot_);
        virtual void execute(void*) = 0;
        virtual bool isFinish()=0;

        virtual ~AbstractTactic();
    protected:
        EvaluationModule& evaluation;
        Robot& robot;
        TaskSharedPtr currentTask;
        double bestScore;
        const log4cxx::LoggerPtr log;


};

#endif // ABSTRACTTACTIC_H
