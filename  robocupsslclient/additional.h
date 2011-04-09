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

#include "boost/tuple/tuple.hpp"
#include "Vector2d/Vector2D.h"
#include "RotationMatrix/RotationMatrix.h"
#include <boost/math/special_functions/fpclassify.hpp>


//dokladnosc polozenia robota
//lub okreslania czy robot jest u celu itp
#define LOCATION_PRECISION 0.01
#define ROTATION_PRECISION 0.01

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
	const Pose operator-(const Pose & pose){
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
	const Pose operator+(const Pose a){
		return Pose( this->get<0>()+a.get<0>(),
				this->get<1>()+a.get<1>(),
				this->get<2>()+a.get<2>() );
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

typedef std::vector<std::string> strvec;

///@brief Funkcja signum
double sgn(double d);

///@brief Konwertuje zadany kąt w radianach do przedziału -PI...PI
///@return kąt w radianach z przedziału -M_PI..+M_PI
double convertAnglePI(double angle);

///@brief Konwertuje zadany kąt w radianach do przedziału 0..2PI
///@return kąt w radianach z przedziału 0..+2*M_PI
double convertAngle2PI(double angle);

/** @author Kamil Muszyński
 *
 * @brief klasa zawierająca nazwy wszystkich modeli gazebo znajdujących się na planszy
 * */
class Names{
public:
	static strvec & getNames();
private:
	static strvec names_list;
	Names();
	~Names();
};

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

enum what{start, stop};

double measureTime(what what_,struct timespec * startTime);

//struct timeval measureTime(what what_,struct timeval * startTime);

#endif /*ADDITIONAL_H_*/
