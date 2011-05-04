#ifndef ROBOT_H_
#define ROBOT_H_

#include <math.h>
#include <string>

#ifdef GAZEBO
	#include <gazebo/gazebo.h>
#endif

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "../SimControl/SimControl.h"
#include "../Vector2d/Vector2D.h"
#include "../additional.h"
#include "../PidRegulator/PidRegulator.h"

/**
 * @author Maciej Gąbka
 *
 * @brief podstawowa klasa reprezentujaca pojedynczego robota mobilnego
 *
 */
class GameState;

class Robot
{
public:

	enum robotID{
		red0 = -4,
		red1 = -3,
		red2 = -2,
		red3 = -1,
		unknown = 0,
		blue0 = 1,
		blue1,
		blue2,
		blue3
	};

	Robot(const std::string robotName,const std::string posIfaceName);
	virtual ~Robot();
	///metoda zwracajaca unikalna nazwe robota
	std::string getRobotName() const ;
	robotID getRobotID() const;
	static robotID getRobotID(const std::string & robotName);
	static bool isRed(robotID id);
	static bool isBlue(robotID id);
	std::string getPosIfaceName() const;
	/*@brief metoda ustalajaca predkosci liniowa oraz katowa
	 *
     * @param v [in] predkosc liniowa robota w ukl wspolrzednych zw z robotem
     * @param w [in] predkosci katowa robota w ukladzei wsp zw z robotem
	 */
	void setRelativeSpeed(const Vector2D & v,const double & w);
	/// zwraca aktualnie zadane predkosci robota first = V  second = w
	std::pair<Vector2D,double> getDesiredVel() const;
	/// zwraca aktualnie  predkosci z jakimi faktycznie porusza sie robot robota first = V  second = w
	std::pair<Vector2D,double> getVelocity() const;
	bool kick() const ;
	bool kickerReady()const;
	double calculateAngularVel(GameState & gameState,Robot::robotID robotID, Pose targetPosition);
	void stop( );

private :
	PidRegulator pidRegulator;
	///zadana predkosc liniowa
	Vector2D v;
	///zadana predkosc katowa
	double w;
	///zmienna okreslajaca nazwer robota, jednoznacznie
	///identyfikujaca go na boisku
	const std::string robotName;
	const Robot::robotID id;
	///nazwa interfejsu do pobierania pozycji
	std::string posIfaceName;
	const log4cxx::LoggerPtr log;
	///pozycje dlugofalowe ustalane przez stratega
	//Vector2D goTo;
#ifdef GAZEBO
    #ifdef OLD
	///interfejs do symulacji
	 gazebo::PositionIface *posIface;
	 #else
	 libgazebo::PositionIface *posIface;
	 #endif
#endif
	 //biezaca pozycja robota
	 Vector2D position;
	 //biezaca rotacja robota
	 double rot;
	 //czas pomiaru
	 double time;


};

//oblicza zadane sterowanie
//Vector2D calculateVelocity(Vector2D currVel, Pose currPose, Pose targetPose, double deltaTime);
//oblicza zadane sterowanie
//Vector2D calculateVelocity(const Vector2D &currVel, const Pose& currPose,const  Pose & targetPose);
/*@brief oblicza predkosci konieczne do przemieszczenia sie do zadanego celu
 *
 * @param [in] currVel predkosci robota w ukladzie odniesienia zwiazanym z robotem
 * @param [in] targetPose polozenie celu w ukladzie odniesienia zwiazanym z robotem
 */
//Vector2D calculateVelocity(const Vector2D &currVel,const  Pose & targetPose);
Vector2D calculateVelocity(const Vector2D &currVel,const  Pose & currGlobalPose,const  Pose & targetGlobalPose);
//double calculateVelocity(const double vel, const double curr,const  double target);

std::ostream& operator<<(std::ostream& os,const Robot::robotID& id);

#endif /*ROBOT_H_*/
