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

#include <list>
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
		red = -5,
		red0 = -4,
		red1 = -3,
		red2 = -2,
		red3 = -1,
		unknown = 0,
		blue0 = 1,
		blue1,
		blue2,
		blue3,
		blue
	};

	Robot(const std::string robotName,const std::string posIfaceName);
	virtual ~Robot();
	///metoda zwracajaca unikalna nazwe robota
	std::string getRobotName() const ;
	robotID getRobotID() const;
	static robotID getRobotID(const std::string & robotName);
	static bool isRed(robotID id);
	static bool isBlue(robotID id);

	static std::list<robotID> getAllRobots();
	static std::list<robotID> getBlueTeam();
	static std::list<robotID> getRedTeam();

	std::string getPosIfaceName() const;
	/*@brief metoda ustalajaca predkosci liniowa oraz katowa w ukladzie wspo zw z robotem
	 *
     * @param v [in] predkosc liniowa robota w ukl wspolrzednych zw z robotem
     * @param w [in] predkosci katowa robota w ukladzei wsp zw z robotem
	 */
	void setRelativeSpeed(const Vector2D & v,const double & w);
	/*@brief metoda ustalajaca globalna predkosc liniowa oraz katowa
	*
	* @param v [in] predkosc liniowa robota w globalnym ukl wspolrzednch
	* @param w [in] predkosci katowa robota w globalnym ukl wspolrzednch
	*/
	void setGlobalSpeed(const Vector2D & v,const double & w, const double& rot);
	/// zwraca aktualnie zadane predkosci robota first = V  second = w
	std::pair<Vector2D,double> getDesiredVel() const;
	/// zwraca aktualnie  predkosci z jakimi faktycznie porusza sie robot robota first = V  second = w
	std::pair<Vector2D,double> getRelativeVelocity() const;
	bool kick() const ;
	bool kickerReady()const;
	/// double dist wymagana odleglosc od przeszkody
	bool disperse(const double dist );
	Vector2D repulsivePotentialField( const Vector2D positionCoordinate, const Vector2D goal, std::list< Vector2D > obstacles );
	Vector2D repulsivePotentialField( const Vector2D positionCoordinate, std::list< Vector2D > obstacles );

	Vector2D navigationFunction( const Vector2D positionCoordinates, const Vector2D goal, std::list< Vector2D > obstacles);

	//double calculateAngularVel(const Pose & robotPosition, const Pose & targetPosition);
	//double calculateAngularVel(const Pose & robotPosition, const Vector2D & targetPosition);

	double calculateAngularVel( const  Pose & globalRobotPose, const Vector2D & globalTargetPosition );
	double calculateAngularVel( const  Pose & globalRobotPose, const  Pose & globalTargetPose );
	void stop( );


private :

	//Vector2D repulsivePotentialField( Vector2D positionCoordinate,  std::list< Vector2D > obstacles );

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
	 //do wyznaczania predkosci obrotowej
	 double oldAlfaToCel;


};

/*@brief oblicza predkosci konieczne do przemieszczenia sie do zadanego celu
 *
 * @param [in] currVel predkosci robota w ukladzie odniesienia zwiazanym z robotem
 * @param [in] targetPose polozenie celu w ukladzie odniesienia zwiazanym z robotem
 */
Vector2D calculateVelocity(const Vector2D &currVel,const  Pose & currGlobalPose,const  Pose & targetGlobalPose);

std::ostream& operator<<(std::ostream& os,const Robot::robotID& id);

#endif /*ROBOT_H_*/
