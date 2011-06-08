/*
 * RRTPlanner.h
 *
 *  Created on: 2009-12-17
 *      Author: Maciej Gąbka
 */

#ifndef RRTPLANNER_H_
#define RRTPLANNER_H_

//#define DEBUG
#include "../GameState/GameState.h"
#include "../Config/Config.h"
#include "../additional.h"
#include "../TestRRT/TestRRT.h"
#include "RRTNode.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <list>
#include <limits>
#include <boost/random.hpp>

//czas o jaki przewidujemy ruch przeciwnika do przodu
//#define PREDICTION_TIME 0.01 //[sek]

#define GOALPOSE 1
#define WAYPOINT 2
#define RANDOMPOINT 3

class RRTPlanner {
	friend class TestRRT;
public:

enum ErrorCode{
    //mamy plan jak dojechac do celu
    Success=1,
    //robot u celu
    RobotReachGoalPose=0,
    //cel jest w obrebie przeszkody
    TargetInsideObstacle=-100,
    //sterowany robot ulegl koliji
    RobotCollision=-99,
    //pkt docelowy jest poza zasiegiem
    BadTarget=-98,
};

	/**
	 * @brief tworzy obiekt plannera RRT
	 *
	 * @param[in] goalProb prawdopodobienstwo wyboru punktu kierujacego na cel
	 * @param[in] robotName nazwa robota dla ktorego planujemy sciezke
	 * @param[in] withObsPrediction czy przewidujemy polozenie przeszkod
	 * @param[in] currState biezacy stan gry na planszy
	 * @param[in] goalPose pozycja docelowa(razem z rotacja)
	 * @param[in] path sciezka z poprzedniego kroki algorytmu
	 * @param[in] simTime_ biezacy czas symulacji
	 * @param[in] timeMeasur -> czy mierzymy czas wykonania algorytmu rrt
	 *
	 * @return
	 */
	RRTPlanner(const double goalProb,const std::string robotName,bool withObsPrediction,const GameStatePtr currState,
            const Pose goalPose,std::list<Pose> * path, double simTime_, bool timeMeasure);

	RRTPlanner(const double goalProb,const std::string robotName,bool withObsPrediction,const GameStatePtr currState,
	            const Pose goalPose,std::list<Pose> * path, double simTime_, bool timeMeasure, bool analyseAllField);

	GameStatePtr getNearestState();
	/* zwraca wierzcholek docelowy dla robota
	 * wierzcholek spelnia nastepujace warunki
	 * 1.Wierzcholek jest bezpośrednio osiagalny przez robota
	 * 2.Wierzcholek jest na ścieżce prowadzącej do celu
	 * 3.Spośród wierzchołków spelniających 1 i 2 jest  wierzcholkiem najblizej celu
	 */
	GameStatePtr getNextState();
	/**
	* @brief uruchamia algorytm rrt zwraca true jesli alg zostal poprawnie uruchomiony
	* false gdy aktualnie robot jest w przeszkodzie.
	*
	* @return
	*/
	ErrorCode run(double deltaSimTime);
	/*
	 * @brief zapis drzewa do pliku w formie dokumentu xml
	*/
	int serializeTree(const char * fileName,int serializedTrees);

	inline bool isNewPlanFound(){
		return this->foundNewPlan;
	}

	inline bool getDistToNearestObs(){
		return this->distToNearestObs;
	}
	virtual ~RRTPlanner();

private:
	/**
	 * @brief zwraca kolejny punkt docelowy.(losowy lub celu).
	 *
	 * @param[in] goalPose punkt docelowy
	 * @return
	 */
private:	Pose choseTarget( Pose goalPose, int * targetType , std::list<Pose>* wayPoints);
	/**
	 * @brief zwraca element poddrzewa(od currNode) znajdujacy sie najblizej celu.
	 *
	 * @param[in] targetState punkt docelowy
	 * @param[in] currNode elt poddrzewa od ktorego zaczynamy przeszukiwac
	 * @param[in] distance dotychczasowa najmniejsza odleglość
	 * @return
	 */
private:	RRTNodePtr findNearest(const Pose & targetPose,RRTNodePtr& currNode);
	/**
	 * @brief zwraca element drzewa znajdujacy sie najblizej celu.
	 *
	 * @param[in] punkt docelowy
	 * @return
	 */
private:	RRTNodePtr findNearestState(const Pose & targetPose);
	/**
	 * @brief zwraca element drzewa znajdujacy sie najblizej celu.
	 *
	 * @param[in] punkt docelowy
	 * @return
	 */
private:	RRTNodePtr findNearestToTargetState();
	/**
	 * @brief zwraca element drzewa znajdujacy sie najblizej celu ale aktualnie osiagalny.
	 *
	 * @param[in] punkt docelowy
	 * @return
	 */
private:	RRTNodePtr findNearestAttainableState(const Pose & targetPose);

private:	RRTNodePtr findNearestAttainableState(const Pose & targetPose,RRTNodePtr currNode);
	/**
	 * @brief zaczynajac od zadanego stanu zwraca stan losowy.
	 *
	 * @param[in]
	 * @return
	 */
private:	Pose getRandomPose(Pose currentPose);
	/**@brief zwraca dystans pomiedzy currPose  a najblizszym robotem w currState
	 *
	 */
private:	double distanceToNearestObstacle(const GameStatePtr & currState, const Pose &targetPose);
	/**@brief zwraca dystans pomiedzy currPose  a najblizsza przeszkoda w kolekcji obstacles
	 *
	 */
public:	    double distanceToNearestObstacle(const Pose &currPose);
	/**
	 * @brief tworzy kolekcje przeszkod w zaleznosci od odleglosci od robota
	 *
	 * @param[in] robotPose aktualna pozycja robota podczas gry
	 * @return
	 */
private:	void initObstacles(const Pose& robotPose );
	/*@brief zwraca losowa pozycje, uzalezniona od biezacej pozycji robota,
	 * jego aktualnej predkosci i maxymalnego wychylenia
	 *
	 */
private:	Pose getRandomPose(const Pose currentPose,Vector2D velocity, double deltaVel);
	/**
 	 * @brief zwraca stan losowy losowany z rozkladem rownomiernym z przestrzeni stanow.
 	 *
 	 * @return
 	 */
public: Pose getRandomPose();

/**
	 * @brief dokonuje ekspansji stanu w kierunku celu, zwraca stan pusty jesli nastapi kolizja
	 *
	 * @param[in] currState aktualny stan planszy
	 * @param[in] targetPose pozycja docelowa
	 * @return empty pointer is there is a collision with obstacle or new gameState extended from currState
	 */
private:	GameStatePtr extendState(const GameStatePtr & currState,const Pose &targetPose,const double robotReach);
	/**
	 * @brief dokonuje przewiduje polozenie przeszkod w nastepnym kroku algorytmu
	 * przewidywane polozenie zapisyje w
	 *
	 * @param[in] currState aktualny stan planszy
	 * @param[in] deltaSimTime szacowany czas pomiedzy uruchomieniami algorytmu
	 */
private:	void evaluateEnemyPositions(const GameStatePtr & currState,const double deltaSimTime);
	/**
	 * @brief sprawdza czy cel nie lezy w obrebie przeszkody
	 *  zwraca true jesli nastapi kolizja

	 * @param[in] targetPose pozycja docelowa
	 * @param[in] safetyMarigin margines bezpieczenstwa o jaki powiekszamy przeszkode
	 * @param[in] checkAddObstacles jesli true, to sprawdza tez kolizje z przewidywanymi polozeniami przeszkod
	 * @return false if there is a collision with  an obstacle; true if everything is OK
	 */
private:	bool isTargetInsideObstacle(const Pose &targetPose,double safetyMarigin,bool checkAddObstacles = false);
	/**
	 * @brief sprawdza czy targetPose jest bezposrednio osiagalna z currPose
	 *
	 * @param[in] currState aktualny stan planszy
	 * @param[in] targetPose pozycja docelowa
	 * @return false if there is a collision with  an obstacle; true if everything is OK
	 */
private:	bool checkTargetAttainability(const Pose &currPose,const Pose &targetPose,bool checkAddObstacles = true);

private:
	//nazwa modelu robota dla ktorego wyznaczamy punkt docelowy
	const std::string robotName;
	const Robot::robotID robotId;
	//stan od ktorego zaczynamy budowac drzewo
	RRTNodePtr root;
	//stan najblizej celu
	RRTNodePtr nearest;
	//czas ostatniego uruchomienia RRT
	//mierzony w sekundach simTime
	//static double lastSimTime;
	double deltaSimTime;
	//czy znaleziono nowy plan
	//znalezienie nowego planu to znalezienie
	//stanu odleglego od celu o max simTime_
	bool foundNewPlan;

	double distToNearestObs;


	//roboty przeszkody posortowane wzgledem odleglosci od sterowanego robota
	std::vector<Pose> obstacles;
	//przewidywane polozenie przeszkod za deltaSIMTime
	std::vector<Pose> predictedObstaclesPos;

	//stan do ktorego ma dojechac robot w wyniku dzialania RRT
	GameStatePtr resultState;
	//wezel koncowy do ktorego ma dojechac robot
	RRTNodePtr resultNode;

	//sciezka wybrana w poprzednim kroku
	std::list<Pose> * path;
	//4 wybrane punkty ze sciezki path
	std::list<Pose> wayPoints;
	//odleglosc najblizszego wezla drzewa rrt od celu
	double shortestDist;
    //wewnetrzna flaga okreslajaca czy juz dojechano do celu
    bool finish;
    //czy cel jest bezposrednio osiagalny
    bool goDirectToTarget;

	//promień okręgu w jakim losujemy kolejny losowy stan
	static const double randomStateReach;
    //maksymalna liczba potomkow korzenia drzewa RRT
	static const unsigned int maxRootChildren = 4;
	//ograniczenie na maksymalna liczbe wezłów w drzewie
	static const unsigned int maxNodeNumber=150;
    //margines bezp przy wyznaczaniu sciezki
    //o tyle powiekszamy promien robota przy wyznaczaniu sciezki
    static const double SAFETY_MARGIN = 0.09;

	//pozycja docelowa robota
	const Pose goalPose;
    //czy przewidujemy ruch przeszkody
	const bool obsPredictionEnabled;
    //prawdopodobienstwo wyboru punktu kierujacego na cel
	const double toTargetLikelihood;
	//czas z sumulacji ktorego dotyczy drzewo
	const double simTime;
    const log4cxx::LoggerPtr logger;

    bool blockedGoalPose;
    const double maxXvalue;
    const double minXvalue;
    const double maxYvalue;
    const double minYvalue;
};


std::ostream& operator<<(std::ostream& out, enum RRTPlanner::ErrorCode errcode );

#endif /* RRTPLANNER_H_ */
