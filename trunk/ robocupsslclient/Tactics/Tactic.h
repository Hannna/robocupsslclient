#ifndef Tactic_H
#define Tactic_H

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


/*
Active Tactics
shoot (Aim | Noaim | Deflect hrolei)
steal [hcoordinatei]
clear
active def [hcoordinatei]
pass hrolei
dribble to shoot hregioni
dribble to region hregioni
spin to region hregioni
receive pass
receive deflection
dribble to position hcoordinatei hthetai
position for start hcoordinatei hthetai
position for kick
position for penalty
charge ball


Non-Active Tactics
position for loose ball hregioni
position for rebound hregioni
position for pass hregioni
position for deflection hregioni
defend line hcoordinate-1i hcoordinate-2i hmin-disti hmax-disti
defend point hcoordinate-1i hmin-disti hmax-disti
defend lane hcoordinate-1i hcoordinate-2i
block hmin-disti hmax-disti hside-prefi
mark horolei (ball | our goal | their goal | shot)
goalie
stop
velocity hvxi hvyi hvthetai
position hcoordinatei hthetai

*/

#include "../Task/Task.h"
#include "../Robot/Robot.h"
#include "../Thread/Thread.h"
class EvaluationModule;

class Tactic : public Thread
{
    public:
		typedef
			enum predicate_{
				start_from_kickoff = 0x01,

			} predicate;

        Tactic(Robot & robot_);
        virtual bool isFinish()=0;
        void stopTactic(){
        	this->stop = true;
        }

    	void markParam(predicate p);
    	/*@brief zdejmuje dany parametr
    	 *
    	*/
    	void unmarkParam(predicate p);

        virtual ~Tactic();
    protected:
        bool stop;
        virtual void execute(void*) = 0;
        EvaluationModule& evaluation;
        Robot& robot;
        TaskSharedPtr currentTask;
        double bestScore;
        const log4cxx::LoggerPtr log;
        int predicates;
        bool finished;


};

#endif // Tactic_H
