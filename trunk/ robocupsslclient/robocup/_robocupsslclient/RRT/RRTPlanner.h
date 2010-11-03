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
#include "RRTNode.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <list>
#include <limits>
#include <boost/random.hpp>

//margines bezp przy wyznaczaniu sciezki
//o tyle powiekszamy roboty przy wyznaczaniu sciezki
#define SAFETY_MARGIN 0.05

#define PREDICTION_TIME 0.01 //[sek]

class RRTPlanner {
public:
	/**
	 * @brief tworzy obiekt plannera RRT
	 *
	 * @param[in] goalProb prawdopodobienstwo wyboru punktu kierujacego na cel
	 * @param[in] robotName nazwa robota dla ktorego planujemy sciezke
	 * @param[in] goalPose pozycja docelowa(razem z rotacja)
	 * @return
	 */
	RRTPlanner(const double goalProb,const std::string robotName,const GameStatePtr currState,const Pose goalPose,std::list<Pose> * path);
	GameStatePtr getNearestState();
	/* zwraca wierzcholek docelowy dla robota
	 * wierzcholek spelnia nastepujace warunki
	 * 1.Wierzcholek jest bezpośrednio osiagalny przez robota
	 * 2.Wierzcholek jest na ścieżce prowadzącej do celu
	 * 3.Spośród wierzchołków spelniających 1 i 2 jest  wierzcholkiem najblizej celu
	 */
	GameStatePtr getNextState();
	/*
	 * @brief zapis drzewa do pliku w formie dokumentu xml
	*/
	int serializeTree(const char * fileName,int serializedTrees);
	virtual ~RRTPlanner();
private:
	/**
	 * @brief zwraca kolejny punkt docelowy.(losowy lub celu).
	 *
	 * @param[in] goalPose punkt docelowy
	 * @return
	 */
	Pose choseTarget(Pose goalPose,Vector2D velocity,const Pose nearestPose , const double obstacleDist);
	/**
	 * @brief zwraca element poddrzewa(od currNode) znajdujacy sie najblizej celu.
	 *
	 * @param[in] targetState punkt docelowy
	 * @param[in] currNode elt poddrzewa od ktorego zaczynamy przeszukiwac
	 * @param[in] distance dotychczasowa najmniejsza odleglość
	 * @return
	 */
	RRTNodePtr findNearest(const Pose & targetPose,RRTNodePtr currNode);
	/**
	 * @brief zwraca element drzewa znajdujacy sie najblizej celu.
	 *
	 * @param[in] punkt docelowy
	 * @return
	 */
	RRTNodePtr findNearestState(const Pose & targetPose);
	RRTNodePtr findNearestToTarget(RRTNodePtr currNode);
	/**
	 * @brief zwraca element drzewa znajdujacy sie najblizej celu.
	 *
	 * @param[in] punkt docelowy
	 * @return
	 */
	RRTNodePtr findNearestToTargetState();

	RRTNodePtr findNearestAttainableState(const Pose & targetPose);
	RRTNodePtr findNearestAttainableState(const Pose & targetPose,RRTNodePtr currNode);
	/**
	 * @brief zaczynajac od zadanego stanu zwraca stan losowy.
	 *
	 * @param[in]
	 * @return
	 */
	Pose getRandomPose(Pose currentPose);
	/**
	 * @brief zaczynajac od zadanego stanu zwraca stan losowy. Uwzglednia aktualna predkość i dopuszczalna zmienę prędkości
	 * w jednym kroku działania algorytmu.
	 *
	 * @param[in] currentPose
	 * @param[in] velocity predkosc robota w aktualnym punkcie
	 * @param[in] deltaVel dopuszczalna zmiana predkosci
	 * @return
	 */
	//static	Pose getRandomPose(const Pose currentPose,Vector2D velocity, double deltaVel);

	double distanceToNearestObstacle(const GameStatePtr & currState,const Pose &targetPose);
	/**
	 * @brief tworzy kolekcje przeszkod w zaleznosci od odleglosci od robota
	 *
	 * @param[in] robotPose aktualna pozycja robota podczas gry
	 * @return
	 */
	void initObstacles(const Pose& robotPose );

	Pose getRandomPose(const Pose currentPose,Vector2D velocity, double deltaVel);
	/**
	 * @brief zwraca stan losowy losowany z rozkladem rownomiernym z przestrzeni stanow.
	 *
	 * @return
	 */
public:	Pose getRandomPose();
	/**
	* @brief uruchamia algorytm rrt zwraca true jesli alg zostal poprawnie uruchomiony
	* false gdy aktualnie robot jest w przeszkodzie.
	*
	* @return
	*/
	bool run(GameStatePtr currState,double deltaSimTime);
	/**
	 * @brief dokonuje ekspansji stanu w kierunku celu, zwraca stan pusty jesli nastapi kolizja
	 *
	 * @param[in] currState aktualny stan planszy
	 * @param[in] targetPose pozycja docelowa
	 * @return empty pointer is there is a collision with obstacle or new gameState extended from currState
	 */
	GameStatePtr extendState(const GameStatePtr & currState,const Pose &targetPose,const double robotReach);
	/**
	 * @brief dokonuje przewiduje polozenie przeszkod w nastepnym kroku algorytmu
	 *
	 * @param[in] currState aktualny stan planszy
	 * @param[in] deltaSimTime szacowany czas pomiedzy uruchomieniami algorytmu
	 */
	void evaluateEnemyPositions(const GameStatePtr & currState,const double deltaSimTime);
	/**
	 * @brief sprawdza czy przemieszczenie do pozycji targetPose nie lezy w obrebie przeszkody
	 *  zwraca true jesli nastapi kolizja

	 * @param[in] targetPose pozycja docelowa
	 * @param[in] safetyMarigin margines bezpieczenstwa o jaki powiekszamy przeszkode
	 * @return false if there is a collision with  an obstacle; true if everything is OK
	 */
	bool checkCollisions(const Pose &targetPose,double safetyMarigin,bool checkAddObstacles = false);
	/**
	 * @brief sprawdza czy targetPose jest bezposrednio osiagalna z currPose
	 *
	 * @param[in] currState aktualny stan planszy
	 * @param[in] targetPose pozycja docelowa
	 * @return false if there is a collision with  an obstacle; true if everything is OK
	 */
	bool checkAttainability(const Pose &currPose,const Pose &targetPose,bool checkAddObstacles = true);

 private:
	//stan od ktorego zaczynamy budowac drzewo
	RRTNodePtr root;
	//prawdopodobienstwo wyboru punktu kierujacego na cel
	double p;
	//nazwa modelu robota dla ktorego wyznaczamy punkt docelowy
	std::string robotName;
	//promień okręgu w jakim losujemy kolejny losowy stan
	static const double randomStateReach;
	//pozycja docelowa robota
	const Pose goalPose;
	//stan najblizej celu
	RRTNodePtr nearest;
	//czas ostatniego uruchomienia RRT
	//mierzony w sekundach simTime
	//static double lastSimTime;
	double deltaSimTime;
	Pose (*getGoalPose)();
	//roboty przeszkody posortowane wzgledem odleglosci od sterowanego robota
	std::vector<Pose> obstacles;
	std::vector<Pose> addObstacles;
	//stan do ktorego ma dojechac robot w wyniku dzialania RRT
	GameStatePtr resultState;
	RRTNodePtr resultNode;
	//sciezka wybrana w poprzednim kroku
	std::list<Pose> * path;
	double shortestDist;

};

#endif /* RRTPLANNER_H_ */
