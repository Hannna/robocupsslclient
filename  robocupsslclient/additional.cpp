#include "additional.h"
#include "Config/Config.h"
#include <math.h>
#include <boost/assert.hpp>
#include <limits>

#include <libxml/tree.h>
#include <libxml/parser.h>
#include <libxml/xmlmemory.h>
#include <libxml/parser.h>
#include <libxml/xpath.h>
#include <libxml/xpathInternals.h>

bool equealDouble(const double & a, const  double & b) {
    return std::fabs(a - b) < std::numeric_limits<double>::epsilon();
}
/*
strvec Names::names_list;

strvec & Names::getNames()
{
	static Names names;
	return names_list;
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
*/


Pose Pose::transform(const Vector2D& distance,const RotationMatrix & rm) const {
	Pose tmp(this->get<0>()-distance.x,this->get<1>()-distance.y,this->get<2>() );
	return rm.Inverse()*tmp;
}

Pose Pose::translation(const Vector2D& distance) const {
	Pose tmp(this->get<0>()-distance.x,this->get<1>()-distance.y,this->get<2>() );
	return tmp;
}

//*****************************************************

Region::Region(const Vector2D lbc_, const Vector2D ruc_): lbc(lbc_),ruc(ruc_) {
	;
}

Vector2D Region::getMiddle(){
	return Vector2D( (ruc.x + lbc.x)/2 , (ruc.y + lbc.y)/2 );
}

Region::~Region(){

};


std::ostream& operator<<(std::ostream& os,const StraightLine& line){
	return os<<"line goes through points  "<<line.getP1()<<" "<<line.getP2()<<"  A "<<line.getA()<<" B "<<line.getB()<<" C"<<line.getC();
}

StraightLine::StraightLine(const Vector2D p1_,const  Vector2D p2_): p1(p1_),p2(p2_) {

	/*
	double A,B,C;
	//jesli jest to prosta typu x=A
	if( fabs(currPose.get<0>()-targetPose.get<0>() ) < 0.001 ){
		A=1;
		B=0;
		C=-currPose.get<0>();
	}
	//rownanie prostej laczacej robota i pkt docelowy
	else{
		A = ( currPose.get<1>()-targetPose.get<1>() )/(currPose.get<0>()-targetPose.get<0>());
		B = -1;
		C=A*(-targetPose.get<0>()) + targetPose.get<1>();
	}*/

	//jesli jest to prosta typu x=A
	if( fabs(p1.x - p2.x ) < 0.001 ){
		A=1;
		B=0;
		C= -p1.x;
	}
	//rownanie prostej laczacej robota i pkt docelowy
	else{
		A = -1.0*( p1.y - p2.y )/(p1.x - p2.x );
		B = 1.0;
		C=(-1.0)*A*( p2.x ) - p2.y;
	}

	if( fabs(A*p1.x +B*p1.y +C) > 0.01 ){
		std::cout<<"ERROR"<<*this<<" with point "<< p1 <<" err ="<<fabs(A*p1.x +B*p1.y +C) <<std::endl;
		exit(0);
	}

	if( fabs(A*p2.x +B*p2.y +C) > 0.01 ){
		std::cout<<"ERROR"<<*this<<" with point "<< p2 <<" err ="<<fabs(A*p2.x +B*p2.y +C)<<std::endl;
		exit(0);
	}
	//assert( fabs(A*p2.x +B*p2.y +C) < 0.01);
}

double StraightLine::distFromPoint( const Vector2D o ){
	return fabs( A*o.x + B*o.y +C )/sqrt(pow(A,2)+pow(B,2) );
}

StraightLine::~StraightLine(){

}

std::ostream& operator<<(std::ostream& os,const Region& region){
	os<<"left bottom corner "<<region.lbc <<" right upper corner "<<region.ruc;
	return os;
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

double calculateAngleToTarget(const Pose &currRobotPose,const Pose &targetPose ){
	const double robotRotation = currRobotPose.get<2>();
	RotationMatrix rm(robotRotation);
	//pozycja celu w ukladzie wsp zw z robotem
	Pose targetRelativePose = targetPose.transform( currRobotPose.getPosition() , rm);
	//wektor EB gdzie E to idealna pozycja pilki kiedy jest przechwycona przez robota a B pozycja pilki
	//Vector2D eb (ballRelativePose.get<0>(), ballRelativePose.get<1>() - 0.08 );
	//idealna rotacja robota do celu
	//const double angle = convertAnglePI(atan2(t.get<1>(),t.get<0>()) -M_PI/2.0);
	//stara wersja
	//Vector2D oy(0.0,1.0);
	//const double angle = eb.angleTo( oy );
	return convertAnglePI(atan2(targetRelativePose.get<1>(),targetRelativePose.get<0>()) -M_PI/2.0);
}

double calculateProperAngleToTarget(const Pose &currRobotPose,const Pose &targetPose ){
	const double robotRotation = currRobotPose.get<2>();
	RotationMatrix rm0(0);
	//pozycja celu w ukladzie wsp zw z robotem
	Pose targetRelativePose = targetPose.transform( currRobotPose.getPosition() , rm0);
	return convertAnglePI(atan2(targetRelativePose.get<1>(),targetRelativePose.get<0>()) -M_PI/2.0);
}
double sgn(double d){
	if (d > 0) return 1.0;
	if (d == 0) return 0.0;
	return -1.0;
}

template <>
double euclideanNorm<Vector2D>(Vector2D t1, Vector2D t2 ){
	return sqrt( pow(t1.x-t2.x,2 )  + pow(t1.y-t2.y,2 ) );
}

std::ostream& operator<<(std::ostream& os,const Pose& pose){
	os<<"x="<<pose.get<0>()<<" y="<<pose.get<1>()<<" yaw="<<pose.get<2>();
	return os;
}

//return time diff in ms
double measureTime(what what_,struct timespec * startTime){
	//static struct timeval startTime;
	//struct timeval  diff;
	//bzero(&diff, sizeof(struct timeval));
	if(what_==start_measure){
	    clock_gettime(CLOCK_REALTIME, startTime);
		return 0;
	}
	else if(what_==stop_measure)	{
	    struct timespec endTime;
		//struct timespec diff;
		//gettimeofday(&endTime, NULL);
		clock_gettime(CLOCK_REALTIME, &endTime);

		long nanosec = endTime.tv_nsec - startTime->tv_nsec;
		long sec = endTime.tv_sec - startTime->tv_sec;

		if( (sec!=0) && (nanosec < 0 ) ){
            sec--;
            nanosec+=1E9;
		}
		double ms = 1000*sec;
		return  ms + (double)(nanosec/1E6);

	}

	return -1;

}


/**
 * example4:
 * @filename:		the input XML filename.
 * @xpathExpr:		the xpath expression for evaluation.
 * @value:		the new node content.
 *
 * Parses input XML file, evaluates XPath expression and update the nodes
 * then print the result.
 *
 * Returns 0 on success and a negative value otherwise.
 */
std::list<std::string> getRobotNames(const char* filename, const xmlChar* xpathExpr) {
	std::list<std::string> robots;
	xmlDocPtr doc;
    xmlXPathContextPtr xpathCtx;
    xmlXPathObjectPtr xpathObj;

    assert(filename);
    assert(xpathExpr);

    /* Load XML document */
    doc = xmlParseFile(filename);
    if (doc == NULL) {
	fprintf(stderr, "Error: unable to parse file \"%s\"\n", filename);
		return robots;
    }


    /* Create xpath evaluation context */
    xpathCtx = xmlXPathNewContext(doc);
    if(xpathCtx == NULL) {
        fprintf(stderr,"Error: unable to create new XPath context\n");
        xmlFreeDoc(doc);
        return robots;
    }

    /* Evaluate xpath expression */
    xpathObj = xmlXPathEvalExpression(xpathExpr, xpathCtx);
    if(xpathObj == NULL) {
        fprintf(stderr,"Error: unable to evaluate xpath expression \"%s\"\n", xpathExpr);
        xmlXPathFreeContext(xpathCtx);
        xmlFreeDoc(doc);
        return robots;
    }

    //std::cout<<"xpathObj strinVal "<<xpathObj->stringval<<std::endl;


    int size;
    int i;

    size = (xpathObj->nodesetval) ? xpathObj->nodesetval->nodeNr : 0;

	std::cout<<" size " << size<<std::endl;
    for(i = size - 1; i >= 0; i--) {
    	assert(xpathObj->nodesetval->nodeTab[i]);
    	//std::cout<<" xpathObj->nodesetval->nodeTab[i] name " << xpathObj->nodesetval->nodeTab[i]->name<<std::endl;
    	//std::cout<<" xpathObj->nodesetval->nodeTab[i] name " <<
    	//		xmlGetProp(xpathObj->nodesetval->nodeTab[i], BAD_CAST "name") <<std::endl;
    	if( xmlStrncmp ( xmlGetProp(xpathObj->nodesetval->nodeTab[i], BAD_CAST "name"), BAD_CAST "red",3 )==0 ||
    			xmlStrncmp (  xmlGetProp(xpathObj->nodesetval->nodeTab[i], BAD_CAST "name"), BAD_CAST "blue",3 )==0)
    		robots.push_back( (const char*)  xmlGetProp(xpathObj->nodesetval->nodeTab[i], BAD_CAST "name") );
    }


    /* update selected nodes */
    //update_xpath_nodes(xpathObj->nodesetval, value);


    /* Cleanup of XPath data */
    xmlXPathFreeObject(xpathObj);
    xmlXPathFreeContext(xpathCtx);

    /* dump the resulting document */
    //xmlDocDump(stdout, doc);


    /* free the document */
    xmlFreeDoc(doc);

    return robots;
}

/*
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
*/
