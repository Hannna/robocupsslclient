/*
 * GameState.h
 *
 *  Created on: 2009-05-12
 *      Author: Maciej Gąbka
 */

#ifndef GAMESTATE_H_
#define GAMESTATE_H_
#include <map>
#include <iostream>
#include <boost/shared_ptr.hpp>

#include "../Vector2d/Vector2D.h"
#include "../additional.h"


class GameState;
typedef boost::shared_ptr <GameState>  GameStatePtr;
/*@brief klasa zawierajaca dane na ktorych moga operowac planisci
 * zarowno stratedzy jak i niskopoziomowi od unikania kolizji
 *
 */
class GameState {
public: class Robot{
public:
	Robot(Pose pos_=Pose(0.0,0.0,0.0),Vector2D v_=Vector2D(0,0),double w_=0): pos(pos_),v(v_),w(w_){;};
	friend std::ostream& operator<<(std::ostream& os,const Robot& r);
	Pose pos;
	Vector2D v;
	double w;

};
public: class Ball{
public:	Ball(Vector2D pos_=Vector2D(0,0),Vector2D v_=Vector2D(0,0)): pos(pos_),v(v_){;};
	Vector2D pos;
	Vector2D v;
};
public:
	GameState();
	GameState(const GameState& gameState);
	void updateRobotData(std::string name,Pose pose,Vector2D v=Vector2D(0,0),double w=0);
	void updateRobotVel(std::string name,std::pair<Vector2D ,double> vel);
	void updateBallData(Vector2D pos,Vector2D v);
	void updateSimTime(double simTime);
	Pose getRobotPos(std::string name);
	/*@brief zwraca pozycje wszystkich robotwo poza podanym jako argument
	 *
	 */
	std::vector<Pose> getEnemyRobotsPos(std::string robotName);
	Vector2D getRobotVelocity(std::string name);
	double getSimTime();
	Pose getBallPos();
	GameState & operator=(const GameState &gameState);
	friend std::ostream& operator<<(std::ostream& os,const GameState& gs);

	/** funkcje sluzace do zapisu trajektorii robota**/

	/**
	 * @brief statyczna funkcja inicjujaca proces zapisu do pliku trajektorii robota.
	 *
	 * @param file strumien na którym operuja funkcje zapisu do pliku
	 * @param fileName nazwa pliku do ktorego zapisywane sa informacje
	 * @param robotName nazwa robota ktorego ruch jest rejestrowany
	 */
	static	void InitPrint(std::ofstream& file,std::string fileName,std::string robotName);

	/**
	 * @brief statyczna funkcja realizująca proces zapisu do pliku trajektorii robota.
	 *
	 * @param file strumien na którym operuja funkcje zapisu do pliku
	 * @param robotName nazwa robota ktorego ruch jest rejestrowany
	 */

	static	void Print(std::ofstream &file,std::string robotName);
	/**
	 * @brief statyczna funkcja konczaca proces zapisu do pliku trajektorii robota.
	 *
	 * @param file strumien na którym operuja funkcje zapisu do pliku
	 * @param robotName nazwa robota ktorego ruch jest rejestrowany
	 */
	static	void FiniPrint(std::ofstream& file,std::string robotName);

	virtual ~GameState();
private :
	///dane dotyczace robotow
	std::map<std::string,Robot> robots;
	///dane dotyczace pilki
	Ball ball;
	//czas pomiaru stanu gry
	double simTime;
	//do zapisu do pliku w formacie tex
	static 	std::ofstream file;
	static	std::string fileName;
};

#endif /* GAMESTATE_H_ */
