#ifndef ADDITIONAL_H_
#define ADDITIONAL_H_

#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <sstream>
#include <math.h>
#include <sys/time.h>
#include <strings.h>
#include <list>

#include "boost/tuple/tuple.hpp"
#include "Vector2d/Vector2D.h"
#include "RotationMatrix/RotationMatrix.h"
#include <boost/math/special_functions/fpclassify.hpp>

#include <libxml/tree.h>
#include <libxml/parser.h>
#include <libxml/xpath.h>
#include <libxml/xpathInternals.h>

//dokladnosc polozenia robota
//lub okreslania czy robot jest u celu itp
#define LOCATION_PRECISION 0.01
#define ROTATION_PRECISION 0.01


#include <cmath>
#include <limits>

bool equealDouble(const double & a, const  double & b);

enum goalArea{
	top,
	bottom
};

/**
 * @mainpage
 * @author Kamil Muszyński, Maciej Gąbka
 *
 * @ref poprawki_gazebo @n
 * @ref skladnia_plik_swiat
 */

/**
 * @page poprawki_gazebo Poprawki wprowadzone do Gazebo
 *
 * Model.cc
 * - funkcja reset - odkomentowano linie dotyczące zerowania prędkości
 * - funkcja setPose - dodano zerowanie prędkości (Model.cc -> setPose, wywolywane w World.cc)
 *
 * World.cc
 * - dodane obsługi rządania get_all_poses - powoduje ono zapis pozycji wszystkich modeli podanych w model_name
 * simulationData do pliku /tmp/gazebo_poses.txt. Plik ten jest następnie odczytywany przez videoserver. W dużym stopniu
 * przyspiesza to pracę z symulatorem (nie potrzeba pojedynczo odpytywać o każdy z modeli).
 */


const int TEAM_SIZE = 3;

class Pose;
class RotationMatrix;
typedef  std::map<std::string,Pose >::iterator PositionsIt;
/**
 * typ do przechowywania informacji o polozeniu i rotacji robota
 */
class Pose: public boost::tuple<double,double,double>{

public:
	Pose():boost::tuple<double,double,double>(){
		;
	}
	Pose(double x,double y,double rot):boost::tuple<double,double,double>(x,y,rot){
		;
	}
	Pose(Vector2D v,double rot):boost::tuple<double,double,double>(v.x,v.y,rot){
		;
	}
	const Pose operator-(const Pose & pose) const {
		Pose p(this->get<0>()-pose.get<0>(),
				this->get<1>()-pose.get<1>(),
				this->get<2>()-pose.get<2>() );
		return p;
	}
	const Pose operator-=(const Pose &pose){
		this->get<0>()-=pose.get<0>();
		this->get<1>()-=pose.get<1>();
		this->get<2>()-=pose.get<2>();
		return *this;
	}
	const Pose operator*=(const double a){
		this->get<0>()*=a;
		this->get<1>()*=a;
		this->get<2>()*=a;
		return *this;
	}
	const Pose operator*(const double a){
		return Pose( this->get<0>()*a,
				this->get<1>()*a,
				this->get<2>()*a );
	}
	const Pose operator+(const Pose a) const{
		return Pose( this->get<0>()+a.get<0>(),
				this->get<1>()+a.get<1>(),
				this->get<2>()+a.get<2>() );
	}
	const Pose operator+(const Vector2D v) const{
		return Pose( this->get<0>()+v.x,
				this->get<1>()+v.y, this->get<2>() );
	}
	bool operator==(const Pose a){
		if( !equealDouble(this->get<0>() , a.get<0>() ) )
			return false;
		if( !equealDouble(this->get<1>() , a.get<1>() ) )
			return false;
		if( !equealDouble(this->get<2>() , a.get<2>() ) )
			return false;

		return true;
	}
	Vector2D getPosition()const {
		return Vector2D(this->get<0>(),this->get<1>());
	}
	/*@brief transformuje biezace polozenie robota do ukl obroconego o rm i przesunietego o distance
	 *
	 */
	Pose transform(const Vector2D& distance,const RotationMatrix & rm) const ;
	Pose translation(const Vector2D& distance) const ;

	//odleglosc w sensie euklidesowym
	double distance(const Pose &p) const {
		return sqrt( pow(this->get<0>()-p.get<0>(),2) + pow(this->get<1>()-p.get<1>(),2) );
	}
	friend std::ostream& operator<<(std::ostream& os,const Pose& pose);
};

class Region{
	public:
		friend std::ostream& operator<<(std::ostream& os,const Region& region);
		Region(const Vector2D lbc,const  Vector2D ruc);

		Vector2D getMiddle();

		virtual ~Region();
	private:
		Region();
		const Vector2D lbc;
		const Vector2D ruc;
};

class StraightLine{
	public:
		friend std::ostream& operator<<(std::ostream& os,const StraightLine& line);

		StraightLine(const Vector2D p1,const  Vector2D p2);

		Vector2D getPoint1(){
			return p1;
		}

		Vector2D getPoint2(){
				return p2;
		}

		double distFromPoint( const Vector2D p1 );

		double getA() const {
			return A;
		}

		double getB() const {
			return B;
		}

		double getC() const {
			return C;
		}

		Vector2D getP1() const {
			return p1;
		}

		Vector2D getP2() const {
			return p2;
		}

		double angleToOX( ){
			Vector2D w( this->B, -1.0*this->A );

			return w.angleTo( Vector2D(1.0,0.0) );
		}

		double angleToOY( ){
			Vector2D w( this->B, -1.0*this->A );

			return w.angleTo( Vector2D(0.0,1.0) );
		}

		double incline(){
			if(fabs(this->A) < 0.01)
				return 0;
			if(fabs(this->B) < 0.01)
				return M_PI/2.0;
			return atan(-this->A/this->B);
			//return atan2(-this->A,this->B);
			//return atan2(this->B,-this->A);
			//return atan2()
		}

		virtual ~StraightLine();
	private:
		StraightLine();
		double A,B,C;
		const Vector2D p1;
		const Vector2D p2;
};



typedef std::vector<std::string> strvec;

///@brief Funkcja signum
double sgn(double d);

///@brief Konwertuje zadany kąt w radianach do przedziału -PI...PI
///@return kąt w radianach z przedziału -M_PI..+M_PI
double convertAnglePI(double angle);

///@brief Konwertuje zadany kąt w radianach do przedziału 0..2PI
///@return kąt w radianach z przedziału 0..+2*M_PI
double convertAngle2PI(double angle);

//oblicza jaka rotacje musi miec robot aby byc skierowanym na cel
double calculateProperAngleToTarget(const Pose &currRobotPose,const Pose &targetPose );

//oblicza kat o jaki trzeba sie obrocic do celu
double calculateAngleToTarget( const Pose &currRobotPose,const Pose &targetPose );

/*
Pose transformCoordinatesToRobot(currRobotPose){
	currRobotPose=(*currGameState).getRobotPos( robot->getRobotID() );
		robotRotation = currRobotPose.get<2>();
		rm = RotationMatrix(robotRotation);
		t = nextRobotPose.transform( currRobotPose.getPosition() , rm);
}
*/

template <class T>
double euclideanNorm(T t1, T t2 ){
	return sqrt( pow(t1-t2,2 ) );
}
template <>
double euclideanNorm<Vector2D>(Vector2D t1, Vector2D t2 );

/** @author Kamil Muszyński
 *
 * @brief klasa zawierająca nazwy wszystkich modeli gazebo znajdujących się na planszy
 * */
/*
class Names{
public:
	static strvec & getNames();
	~Names();
private:
	static strvec names_list;
	Names();

};
*/

/**
 * function to convert strings to sth else
 * i.e int, unsigned int
 */
template <class T>
bool from_string(T& t,
                 const std::string& s,
                 std::ios_base& (*f)(std::ios_base&))
{
  std::istringstream iss(s);
  return !(iss >> f >> t).fail();
}

enum what{start_measure, stop_measure};

double measureTime(what what_,struct timespec * startTime);

//int xpathTest(const char* filename, const xmlChar* xpathExpr);
//struct timeval measureTime(what what_,struct timeval * startTime);

std::list<std::string> getRobotNames(const char* filename, const xmlChar* xpathExpr);

#endif /*ADDITIONAL_H_*/
