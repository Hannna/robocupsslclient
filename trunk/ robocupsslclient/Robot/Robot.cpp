#include "Robot.h"
#include "../Logger/Logger.h"
#include "../Config/Config.h"
#include "../VideoServer/Videoserver.h"

Robot::Robot(const std::string robotName,const std::string posIfaceName)
{
	this->robotName=robotName;
	this->posIfaceName=posIfaceName;
	this->v=Vector2D(0,0);
	this->w=0;
	this->time=0;

	//alokowanie interfejsu do zmiany pozycji
#ifdef GAZEBO
    #ifdef OLD
	posIface = new gazebo::PositionIface();
	#else
	posIface = new libgazebo::PositionIface();
	#endif
	/// Connect to the libgazebo server
	std::string positionIfaceName=this->robotName+this->posIfaceName;
	std::cout<<"try connect to "<<positionIfaceName<<std::endl;
	SimControl::getInstance().connectGazeboPosIface(posIface,positionIfaceName.c_str());
	std::cout<<"connected to "<<positionIfaceName<<std::endl;
	//Enable the motor
	posIface->Lock(1);
	posIface->data->cmdEnableMotors = 1;
	posIface->Unlock();
#endif
	Videoserver::getInstance().registerRobot(posIface,this->robotName);

}
std::string Robot::getRobotName() const
{
	return this->robotName;
}
std::string Robot::getPosIfaceName() const
{
	return this->posIfaceName;
}
void Robot::setSpeed(Vector2D v, double w)
{
	boost::tuple<double,double,double> currPositions;
	boost::tuple<double,double,double> newVel=boost::tuple<double,double,double> (v.x,v.y,w);
	//SimControl::getInstance().getModelPos(robotName,currPositions);
	/*
	if(this->time!=0){
		double currTime=SimControl::getInstance().getSimTime();
		double deltaTime=currTime-this->time;
		this->pidRegulator.setCmdVel(v,w);
		this->pidRegulator.calculateError( Vector2D( (currPositions.get<0>()-this->position.x)/deltaTime,
				(currPositions.get<1>()-this->position.y)/deltaTime ),
						(currPositions.get<2>()-this->rot)/deltaTime );

		newVel=this->pidRegulator.calculateNewVel();
		this->time=currTime;
	}
	else
		this->time=SimControl::getInstance().getSimTime();
*/
#ifdef GAZEBO
	posIface->Lock(1);
	posIface->data->cmdEnableMotors = 1;
	//posIface->data->cmdVelocity.pos.x = newVel.get<0>();
	posIface->data->cmdVelocity.pos.x = newVel.get<0>();
	posIface->data->cmdVelocity.pos.y = newVel.get<1>();
	posIface->data->cmdVelocity.yaw = newVel.get<2>();
	//posIface->data->cmdVelocity.yaw = 0;
	posIface->Unlock();
#endif
	Logger::getInstance().LogToFile(PATH,"set vel       name=%s\t vx=%lf\t vy=%lf\t,"
				,this->robotName.c_str(),newVel.get<0>(),newVel.get<1>());
}
std::pair<Vector2D,double> Robot::getDesiredVel() const
{
	return std::pair<Vector2D,double>(this->v,this->w);
}
std::pair<Vector2D,double> Robot::getVelocity() const
{
	double vx,vy,w;
	posIface->Lock(1);
	vx = posIface->data->velocity.pos.x;
	vy = posIface->data->velocity.pos.y;
	w = posIface->data->cmdVelocity.yaw;
	posIface->Unlock();


	if(robotName.compare(Config::getInstance().getTestModelName())==0){
		Logger::getInstance().LogToFile(PATH,"robot %s from gazebo vx=%lf\t vy=%lf\t w=%lf,"
					,this->robotName.c_str(),vx,vy,w);
	}

	return std::pair<Vector2D,double>(Vector2D(vx,vy),w);

}

bool Robot::kickerReady(){
       bool ready = false;
       posIface->Lock(1);
       if (posIface->data->cmdVelocity.pos.z <= 0) ready = true;
       posIface->Unlock();
       return ready;
}

bool Robot::kick(){
       if (!kickerReady()) return false;
       posIface->Lock(1);
       //this is never used by other methods, so will be used to fire kicker
       posIface->data->cmdVelocity.pos.z = 1;
       posIface->Unlock();
       return true;
}

Robot::~Robot()
{
	std::cout<<"~Robot "<<this->robotName<<std::endl;
}
Vector2D calculateVelocity(const Vector2D &currVel, const Pose& currPose,const  Pose & targetPose){
	Vector2D newVel;
	newVel.x=calculateVelocity(currVel.x, currPose.get<0>(),targetPose.get<0>());
	newVel.y=calculateVelocity(currVel.y, currPose.get<1>(),targetPose.get<1>());
	return newVel;
}

Vector2D calculateVelocity(const Vector2D &currVel,const  Pose & targetPose){
	Vector2D newVel;
	//std::cout<<"#####################################"<<std::endl;
	//std::cout<<"current robot vel "<<currVel<<std::endl;
	//std::cout<<"curr relative targetPose "<<targetPose<<std::endl;
	//std::cout<<"for X "<<std::endl;
	newVel.x=calculateVelocity(currVel.x, 0,targetPose.get<0>());
	//std::cout<<"for Y "<<std::endl;
	newVel.y=calculateVelocity(currVel.y, 0,targetPose.get<1>());
	//std::cout<<"new robot vel "<<newVel<<std::endl;
	//std::cout<<"#####################################"<<std::endl;
	return newVel;
}
double calculateVelocity(const double vel, const double curr,const  double target){
	double newV;
	//przyspieszenie
	double acc=5;//7.5;//[m/s2]
	//hamowanie
	double dec=1;//9;//[m/s2]
	//jesli predkosc powoduje powiekszanie sie odleglosci miedzy cuur.x oraz target.x to zatrzymaj
		if( vel*target < -0.01){
			//std::cout<<"zly kierunek"<<std::endl;
			newV=0;
		}
		//jesli poruszajac sie z aktualna predkoscia przekroczymy cel
		else if( fabs(target-curr)<= pow(vel,2)/(2*dec)  ){
			//std::cout<<"przekroczymy cel"<<std::endl;
			newV=0;
		}
		//jesli aktualna predkosci jest wieksza niz max
		else if(fabs(vel)>Config::getInstance().getRRTMaxVel()){
			double signum=-1;
			if(vel>0)
				signum=1;
			newV=Config::getInstance().getRRTMaxVel()*signum;
			//std::cout<<"powyzej vmax newV="<<newV<<std::endl;
		}
		else{
			//std::cout<<"trapez"<<std::endl;
			double s=fabs(target-curr);
			double v=sqrt( (3*dec*pow(vel,2)+2*acc*dec*s)/(acc+dec));

			v=fmin(v,Config::getInstance().getRRTMaxVel());
			double signum=-1;
			if(target-curr>0)
				signum=1;
			newV=v*signum;
			//std::cout<<"newV="<<newV<<std::endl;

/*
			//jesli mozemy rozpedzic sie do predkosci maksymalnej oblicz trapezoidalny rozklad predkosci
			if(fabs(target-curr)>= pow(Config::getInstance().getRRTMaxVel(),2)/dec ){
				std::cout<<"rozklad trapez"<<std::endl;
				double signum=-1;
				if(target-curr>0)
					signum=1;
				newV=Config::getInstance().getRRTMaxVel()*signum;
			}
			//oblicz trojkatny rozklad predkosci
			else{
				std::cout<<"rozklad trojkatny"<<std::endl;
				double signum=-1;
				if(target-curr>0)
					signum=1;
				newV=signum*sqrt(fabs(target-curr))/acc;//Config::getInstance().getRRTMaxVel()*signum;
			}
			*/
		}
		return newV;
}
/*
Vector2D calculateVelocity(Vector2D currVel, Pose currPose, Pose targetPose, double deltaTime){
	//przyspieszenie
	double acc=0.5;//[m/s2]
	//hamowanie
	double dec=0.05;//[m/s2]
	Vector2D newV(targetPose.get<0>()-currPose.get<0>(),targetPose.get<1>()-currPose.get<1>());

	//pozcja robota za pewien kwant czasu
	Pose newPose=currPose;
	newPose.get<0>()+=deltaTime*currVel.x;
	newPose.get<1>()+=deltaTime*currVel.y;

	//rozpatrujemy os x

	//jesli predkosc powoduje powiekszanie sie odleglosci miedzy cuur.x oraz target.x to zatrzymaj
	if(fabs(targetPose.get<0>()-currPose.get<0>() ) >  fabs(targetPose.get<0>()-newPose.get<0>() ) ){
		newV.x=std::max<double>(0,currVel.x -dec*deltaTime);
	}
	//jesli predkosc powoduje pojechanie poza cel to zatrzymaj
	else if(fabs(currPose.get<0>() + currVel.x*deltaTime) >  fabs(targetPose.get<0>() ) ){
		newV.x=std::max<double>(0,currVel.x - dec*deltaTime);
	}
	else{
		double factor = Config::getInstance().getRRTMaxVel()/newV.length();
		newV.x=std::min<double>(newV.x+deltaTime*acc,Config::getInstance().getRRTMaxVel());
	}
	//rozpatrujemy os y

	//jesli predkosc powoduje powiekszanie sie odleglosci miedzy cuur.x oraz target.x to zatrzymaj
	if(fabs(targetPose.get<1>()-currPose.get<1>() ) >  fabs(targetPose.get<1>()-newPose.get<1>() ) ){
		newV.y=std::max<double>(0,currVel.y -dec*deltaTime);
	}
	//jesli predkosc powoduje pojechanie poza cel to zatrzymaj
	else if(fabs(currPose.get<1>() + currVel.y*deltaTime) >  fabs(targetPose.get<1>() ) ){
		newV.y=std::max<double>(0,currVel.y - dec*deltaTime);
	}
	else{
		double factor = Config::getInstance().getRRTMaxVel()/newV.length();
		newV.y=std::min<double>(newV.y+deltaTime*acc,Config::getInstance().getRRTMaxVel());
	}

	return newV;

}
*/
