#ifndef ROBOT_H_
#define ROBOT_H_

#include <math.h>
#include <string>
#include <boost/math/special_functions/fpclassify.hpp>

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
#include <fstream>

/**
 * @author Maciej Gąbka
 *
 * @brief podstawowa klasa reprezentujaca pojedynczego robota mobilnego
 *
 */
class GameState;

typedef
struct funcParams_{
	double param;
	std::list< Vector2D > obstacles;
	Vector2D goal;

}funcParams;

double navigationFunctionX( const double x, void * params);

double navigationFunctionY( const double y, void * params);

class Robot
{
public:

	enum robotID{
		red = -8,
		red0 = -7,
		red1 = -6,
		red2 = -5,
		red3 = -4,
		red4 = -3,
		red5 = -2,
		red6 = -1,
		unknown = 0,
		blue0 = 1,
		blue1,
		blue2,
		blue3,
		blue4,
		blue5,
		blue6,
		blue
	};

	Robot(const std::string robotName,const std::string posIfaceName);
	virtual ~Robot();
	//
	static const std::string ifaceName;
	///metoda zwracajaca unikalna nazwe robota
	std::string getRobotName() const ;
	robotID getRobotID() const;
	static robotID getRobotID(const std::string & robotName);

	static bool isRed(robotID id);
	bool isRed( );

	static bool isBlue(robotID id);
	bool isBlue( );

	static std::list<robotID> getAllRobots();
	static std::list<robotID> getBlueTeam();
	static std::list<robotID> getRedTeam();

	double getLastTetaToBall(){
		return this->last_teta_to_ball;
	}

	void setLastTetaToBall( double t){
		this->last_teta_to_ball = t;;
	}

	std::string getPosIfaceName() const;
	/*@brief metoda ustalajaca predkosci liniowa oraz katowa w ukladzie wspo zw z robotem
	 *
     * @param v [in] predkosc liniowa robota w ukl wspolrzednych zw z robotem
     * @param w [in] predkosci katowa robota w ukladzei wsp zw z robotem, w> 0 obrot w kierunku dodatnich katow fi, w< 0 obrot w kierunku ujemnych katow
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
	std::pair<Vector2D,double> getGlobalVelocity() const;
	bool kick( const double force ) const ;
	bool kickerReady()const;
	/// double dist wymagana odleglosc od przeszkody
	bool disperse( const double dist );
	/// jedz do pilki wymagana odleglosc od pilki
	bool goToBall(const double dist );
	//Vector2D repulsivePotentialField( const Vector2D positionCoordinate, const Vector2D goal, std::list< Vector2D > obstacles );
	//Vector2D repulsivePotentialField( const Vector2D positionCoordinate, std::list< Vector2D > obstacles );

	Vector2D navigationFunctionGradient( const Vector2D positionCoordinates, const Vector2D goal, std::list< Vector2D > obstacles);

	Vector2D navigationFunctionGradient2( const Vector2D positionCoordinates, const Vector2D goal, std::list< Vector2D > obstacles, double granularity);

	double navigationFunction( const Vector2D positionCoordinates, const Vector2D goal, std::list< Vector2D > obstacles);

	//double calculateAngularVel(const Pose & robotPosition, const Pose & targetPosition);
	//double calculateAngularVel(const Pose & robotPosition, const Vector2D & targetPosition);

	double calculateAngularVel( const  Pose & globalRobotPose, const Vector2D & globalTargetPosition , double simTime, const bool haveBall );
	double calculateAngularVel( const  Pose & globalRobotPose, const  Pose & globalTargetPose, const double simTime, const bool haveBall );
	double calculateAngularVel( const  Pose & globalRobotPose, const double rotation, const double simTime, const bool haveBall );
	void stop( );
	/*@brief oblicza predkosci konieczne do przemieszczenia sie do zadanego celu
	 *
	 * @param [in] currVel predkosci robota w ukladzie odniesienia zwiazanym z robotem
	 * @param [in] targetPose polozenie celu w ukladzie odniesienia zwiazanym z robotem
	 */
	Vector2D calculateVelocity(const Vector2D &currVel,const  Pose & currGlobalPose,const  Pose & targetGlobalPose);

	void setMaxDcc( const double dcc ){
		this->maxDcc = dcc;
	}
private :
	bool navigateToPose( const double dist, const Vector2D* goalPose, const bool onlyDisperse = false );
	double calculateVelocity(const double vel, const double currPosition,const  double targetPosition);

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
	double maxAcc;
	double maxDcc;
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

	 double lastUpdateTime;
	 double last_tetad;
	 double last_teta;

	 double last_teta_to_ball;

	 std::ofstream file;
	 std::string fileName;

	 std::string file_teta_name;
	 std::ofstream file_teta;

	 std::string file_v_name;
	 std::ofstream file_v;

	 std::string file_xy_name;
	 std::ofstream file_xy;


	 const static int filterSize = 5;
	 int last_w_index;
	 double last_angular_vel[filterSize];


};

boost::tuple< double, double, double > calculateCurwatureVelocity( const double radious,const double maxW );
std::ostream& operator<<(std::ostream& os,const Robot::robotID& id);

#endif /*ROBOT_H_*/
