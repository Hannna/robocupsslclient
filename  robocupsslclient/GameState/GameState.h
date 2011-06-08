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

#include "../Robot/Robot.h"

class GameState;
typedef boost::shared_ptr <GameState>  GameStatePtr;

/*@brief klasa zawierajaca dane na ktorych moga operowac planisci
 * zarowno stratedzy jak i niskopoziomowi od unikania kolizji
 *
 */
class GameState {

public: class RobotState{

public:
	RobotState(Pose pos_=Pose(0.0,0.0,0.0),Vector2D v_=Vector2D(0,0),double w_=0): pos(pos_),v(v_),w(w_){;};
	friend std::ostream& operator<<(std::ostream& os,const RobotState& r);
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

	static Robot::robotID getRobotID(const std::string& robot_name);

	void updateRobotData(std::string name,Pose pose,Vector2D v=Vector2D(0,0),double w=0);
	void updateRobotVel(std::string name,std::pair<Vector2D ,double> vel);

	void updateRobotData(Robot::robotID id,Pose pose,Vector2D v=Vector2D(0,0),double w=0);
	void updateRobotVel(Robot::robotID id,std::pair<Vector2D ,double> vel);

	Vector2D getBallGlobalVelocity( ) const ;
	Pose getBallPos() const ;
	void updateBallData(Vector2D pos,Vector2D v);

	void updateSimTime(double simTime);

	Pose getRobotPos(std::string name) const ;
	Pose getRobotPos(const Robot::robotID id) const ;

	double getSimTime() const ;
	void setSimTime(double simTime_){this->simTime=simTime_;};
	/*@brief zwraca pozycje wszystkich robotwo poza podanym jako argument
	 *
	 */
	std::vector<Pose> getEnemyRobotsPos(const Robot::robotID & id) const ;

	Vector2D getRobotGlobalVelocity(const std::string name) const ;
	Vector2D getRobotGlobalVelocity(const Robot::robotID id) const ;
	double getRobotAngularVelocity(const Robot::robotID id) const ;

	GameState & operator=(const GameState &gameState);
	friend std::ostream& operator<<(std::ostream& os,const GameState& gs) ;
	friend std::ostream& operator<<(std::ostream& os,const GameState* const gs) ;



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

	inline const goalArea getRedGoalArea(){
		return redGoal;
	}
	inline const goalArea getBlueGoalArea(){
		return blueGoal;
	}
private :
	const goalArea redGoal;
	const goalArea blueGoal;
	///dane dotyczace robotow
	std::map<Robot::robotID,RobotState> robots;
	typedef std::map<Robot::robotID,RobotState>::iterator RobotsPoseIt;
	typedef std::map<Robot::robotID,RobotState>::const_iterator RobotsPoseConstIt;
	///dane dotyczace pilki
	Ball ball;
	//czas pomiaru(z symulatora) stanu gry
	double simTime;
//	bool updated;
	//do zapisu do pliku w formacie tex
	static 	std::ofstream file;
	static	std::string fileName;
};

#endif /* GAMESTATE_H_ */
