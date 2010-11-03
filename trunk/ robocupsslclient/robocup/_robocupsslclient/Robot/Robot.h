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

class Robot
{
public:
	Robot(const std::string robotName,const std::string posIfaceName);
	virtual ~Robot();
	///metoda zwracajaca unikalna nazwe robota
	std::string getRobotName() const ;
	std::string getPosIfaceName() const;
	///metoda ustalajaca predkosci liniowa oraz katowa
	void setSpeed(Vector2D v, double w);
	/// zwraca aktualnie zadane predkosci robota first = V  second = w
	std::pair<Vector2D,double> getDesiredVel() const;
	/// zwraca aktualnie  predkosci z jakimi faktycznie porusza sie robot robota first = V  second = w
	std::pair<Vector2D,double> getVelocity() const;
	bool kick();
	bool kickerReady();

private :
	PidRegulator pidRegulator;
	///zadana predkosc liniowa
	Vector2D v;
	///zadana predkosc katowa
	double w;
	///zmienna okreslajaca nazwer robota, jednoznacznie
	///identyfikujaca go na boisku
	std::string robotName;
	///nazwa interfejsu do pobierania pozycji
	std::string posIfaceName;
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
Vector2D calculateVelocity(const Vector2D &currVel, const Pose& currPose,const  Pose & targetPose);
/*@brief oblicza predkosci konieczne do przemieszczenia sie do zadanego celu
 *
 * @param [in] currVel predkosci robota w ukladzie odniesienia zwiazanym z robotem
 * @param [in] targetPose polozenie celu w ukladzie odniesienia zwiazanym z robotem
 */
Vector2D calculateVelocity(const Vector2D &currVel,const  Pose & targetPose);

double calculateVelocity(const double vel, const double curr,const  double target);
#endif /*ROBOT_H_*/
