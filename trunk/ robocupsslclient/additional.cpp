#include "additional.h"
#include "Config/Config.h"
#include <math.h>
#include <boost/assert.hpp>
#include <limits>
/*namespace additional{
	ObjectsID getRobotID(std::string name)
	{
		for (int i = 0; i < 6; i++)
			if (name == robotNames[i])
				return static_cast<ObjectsID>(i);

		std::cout<<"[additional.h - getID] Błędna nazwa modelu! ["+name+"]\n";
		exit(0);
	}
}
*/
strvec Names::names_list;

strvec & Names::getNames()
{
	static Names names;
	return names_list;
}
Pose Pose::transform(const Vector2D& distance,const RotationMatrix & rm) const {
	Pose tmp(this->get<0>()-distance.x,this->get<1>()-distance.y,this->get<2>() );
	return rm.Inverse()*tmp;
}
Names::Names()
{
	strvec::const_iterator ii;

	strvec blueTeam=Config::getInstance().getBlueTeam();
	for(ii=blueTeam.begin();ii!=blueTeam.end();ii++){
			names_list.push_back(*ii);
	}
	strvec redTeam=Config::getInstance().getRedTeam();
	for(ii=redTeam.begin();ii!=redTeam.end();ii++){
		names_list.push_back(*ii);
	}
	names_list.push_back("ball");
}

Names::~Names()
{
}


//*****************************************************

double convertAnglePI(double angle){
	BOOST_ASSERT(fabs(angle)!=std::numeric_limits<double>::infinity());
	double temp_angle=angle;
	double abs_angle = fabs(angle);
	double sgn_angle = (angle > 0 ? 1.0 : -1.0);
	double k = floor(abs_angle / (2*M_PI));
	double _2PI = 2* M_PI;
	if (abs_angle > M_PI){
		if (abs_angle>_2PI)
			angle = angle - (sgn_angle * k * _2PI);		//normowanie kąta - nie większy niż 2PI

		if ((angle) < -M_PI)  angle = _2PI + angle;	//normowanie do przedzialu -PI..PI
		else
			if ((angle) > M_PI)  angle = angle - _2PI;

		//BOOST_ASSERT(angle >=-M_PI && angle <= M_PI);
		if( !(angle >=-M_PI && angle <= M_PI))
			std::cout<<"in angle"<<temp_angle<<" out angle"<<angle<<std::endl;
		return angle;
	}
	else
		return angle;
}


double convertAngle2PI(double angle){
	double abs_angle = fabs(angle);
	double sgn_angle = (angle > 0 ? 1.0 : -1.0);
	double k = floor(abs_angle / (2*M_PI));
	double _2PI = 2* M_PI;
	if (abs_angle > _2PI || angle < 0){
		angle = angle - (sgn_angle * k * _2PI);		//normowanie kąta - nie większy niż 2PI
		if ((angle) < 0)  angle = _2PI + angle;	//angle ujemny! wiec odejmowanie

		BOOST_ASSERT(angle >=0 && angle <= _2PI);
		return angle;
	}
	else
		return angle;
}


double sgn(double d){
	if (d > 0) return 1.0;
	if (d == 0) return 0.0;
	return -1.0;
}

std::ostream& operator<<(std::ostream& os,const Pose& pose){
	os<<"x="<<pose.get<0>()<<" y="<<pose.get<1>()<<" yaw="<<pose.get<2>();
	return os;
}

struct timeval measureTime(what what_,struct timeval * startTime){
	//static struct timeval startTime;
	struct timeval  diff;
	bzero(&diff, sizeof(struct timeval));
	if(what_==start){
		gettimeofday(startTime, NULL);
		return *startTime;
	}
	if(what_==stop)	{
		struct timeval endTime;
		gettimeofday(&endTime, NULL);
		diff.tv_sec = endTime.tv_sec-startTime->tv_sec;

		if(endTime.tv_usec > startTime->tv_usec){
			diff.tv_usec = endTime.tv_usec-startTime->tv_usec;
		}
		else{
			diff.tv_sec-=1;
			diff.tv_usec=1000000-startTime->tv_usec +endTime.tv_usec;
		}
	}

	return diff;

}
