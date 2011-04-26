#include "Robot.h"
#include "../Logger/Logger.h"
#include "../Config/Config.h"
#include "../VideoServer/Videoserver.h"
#include "../GameState/GameState.h"

std::ostream& operator<<(std::ostream& os,const Robot::robotID& id){
	if( id==Robot::red0 ){
		return os<<"red0";
	}
	else if( id==Robot::red1 ){
		return os<<"red1";
	}
	else if( id==Robot::red2 ){
		return os<<"red2";
	}
	else if( id==Robot::blue0 ){
		return os<<"blue0";
	}
	else if( id==Robot::blue1 ){
		return os<<"blue1";
	}
	else if( id==Robot::blue2 ){
		return os<<"blue2";
	}

	return os<<"unknown";
}

Robot::Robot(const std::string robotName_,const std::string posIfaceName) : robotName( robotName_ ), id( Robot::getRobotID(robotName_) )
{
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

    posIface->data->cmdVelocity.pos.x=0;
	posIface->data->cmdVelocity.pos.y=0;
	posIface->data->cmdVelocity.pos.z=0;

	posIface->Unlock();
	Videoserver::getInstance().registerRobot(posIface,this->robotName);
#endif

}
std::string Robot::getRobotName() const
{
	return this->robotName;
}
std::string Robot::getPosIfaceName() const
{
	return this->posIfaceName;
}

void Robot::setRelativeSpeed(const Vector2D & v, const double & w)
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
	posIface->data->cmdVelocity.pos.x = newVel.get<0>();
	posIface->data->cmdVelocity.pos.y = newVel.get<1>();
    posIface->data->cmdVelocity.yaw = newVel.get<2>();

//	posIface->data->cmdVelocity.yaw = (M_PI*newVel.get<2>())/180.0;
//  posIface->data->cmdVelocity.yaw = 0;

    posIface->Unlock();
#endif
	LOG_TRACE(getLoggerPtr("path"),"set vel       name="<<this->robotName.c_str()<<"\t vx="<<newVel.get<0>()<<"\t vy="<<newVel.get<1>()<<"\t" );
}
std::pair<Vector2D,double> Robot::getDesiredVel() const
{
	return std::pair<Vector2D,double>(this->v,this->w);
}
std::pair<Vector2D,double> Robot::getVelocity() const
{
	double vx=0,vy=0,w=0;
#ifdef GAZEBO
	posIface->Lock(1);
	vx = posIface->data->velocity.pos.x;
	vy = posIface->data->velocity.pos.y;
	w = posIface->data->cmdVelocity.yaw;
	w = 180*w/M_PI;
	posIface->Unlock();
#endif

	if(robotName.compare(Config::getInstance().getTestModelName())==0){
		//LOG_DEBUG(getLoggerPtr("path"),"robot "<<this->robotName.c_str()<<" from gazebo vx"=%lf\t vy=%lf\t w=%lf,"
		//			,,vx,vy,w);
		;
	}

	return std::pair<Vector2D,double>(Vector2D(vx,vy),w);

}

bool Robot::kickerReady()const{
	bool ready = false;
#ifdef GAZEBO
	posIface->Lock(1);
	if (posIface->data->cmdVelocity.pos.z <= 0) ready = true;
	posIface->Unlock();
#endif
	return ready;
}

bool Robot::kick()const {
	if (!kickerReady()) return false;
#ifdef GAZEBO
	posIface->Lock(1);
	//this is never used by other methods, so will be used to fire kicker
	posIface->data->cmdVelocity.pos.z = 1;
	posIface->Unlock();
#endif
	return true;
}

Robot::robotID Robot::getRobotID() const {
	return this->id;
}
Robot::robotID Robot::getRobotID(const  std::string & robot_name){
	if( robot_name.compare("red0")==0 ){
		return Robot::red0;
	}
	else if( robot_name.compare("red1")==0 ){
		return Robot::red1;
	}
	else if( robot_name.compare("red2")==0 ){
		return Robot::red2;
	}
	else if( robot_name.compare("blue0")==0 ){
		return Robot::blue0;
	}
	else if( robot_name.compare("blue1")==0 ){
		return Robot::blue1;
	}
	else if( robot_name.compare("blue2")==0 ){
		return Robot::blue2;
	}

	return Robot::unknown;
}


void Robot::stop(  ){
/*
	if( speed.length() > 0.1 ){
		Vector2D result;
		static Vector2D oldResult;

		double Ker=0.5;
		double Ko=20;

		result.x = Ko*( 0 - speed.x) + Ker*( 0 - oldResult.x );
		result.y = Ko*( 0 - speed.y) + Ker*( 0 - oldResult.y );

		oldResult = result;

		this->setRelativeSpeed(result,0);
	}
*/
	this->setRelativeSpeed( Vector2D(0,0),0 );

}

double Robot::calculateAngularVel(GameState & gameState,Robot::robotID robotID, Pose targetPosition){
    //static GameState oldGameState;
    static double oldTetaCel;
    double rotacjaDocelowa=atan2(targetPosition.get<0>(),targetPosition.get<2>());
    double Ker=0.5;
    double Ko=20;
    double currGlobalRobotRot=gameState.getRobotPos( robotID ).get<2>();
    //macierz obrotu os OY na wprost robota
    RotationMatrix rmY(currGlobalRobotRot);
    //macierz obrotu os OY nw wprost robota
    //RotationMatrix rmY(-M_PI/2);
    //pozycja robota w ukladzie wsp zw z plansza
    //Vector2D currentPosition=Videoserver::data.getPosition(this->robotName);
    Pose currRobotPose=gameState.getRobotPos( robotID );
    //pozycja celu w ukladzie wsp zwiazanych z robotem
    Pose reltargetPose=targetPosition.transform(currRobotPose.getPosition(),rmY);
    //targetPosition=rmX.Inverse()*(goToPosition-currentPosition);
    //rotacja do celu
    double currTetaCel=atan2( (reltargetPose.get<1>()) , (reltargetPose.get<0>()));

    //double currTetaCel=atan( (-reltargetPose.get<1>()) / (reltargetPose.get<0>()));

    double angularVel=Ko*(rotacjaDocelowa-currGlobalRobotRot)+ Ker*(oldTetaCel-currTetaCel);

    oldTetaCel=currTetaCel;

    return angularVel;
}

Robot::~Robot()
{
	std::cout<<"~Robot "<<this->robotName<<std::endl;
}
/*
Vector2D calculateVelocity(const Vector2D &currVel, const Pose& currPose,const  Pose & targetPose){
	Vector2D newVel;
	newVel.x=calculateVelocity(currVel.x, currPose.get<0>(),targetPose.get<0>());
	newVel.y=calculateVelocity(currVel.y, currPose.get<1>(),targetPose.get<1>());
	return newVel;
}
*/



double calculateVelocity(const double vel, const double currPosition,const  double targetPosition){
	double newV;
	//przyspieszenie
	double acc=2.26305;//7.5;//[m/s2]
	//hamowanie
	double dec=0.5;//9;//[m/s2]
	//jesli predkosc powoduje powiekszanie sie odleglosci miedzy cuur.x oraz target.x to zatrzymaj
		if( vel*targetPosition < -0.01){
			newV=0;
		}
		//jesli poruszajac sie z aktualna predkoscia przekroczymy cel
		else if( fabs(targetPosition-currPosition)<= pow(vel,2)/(2*dec)  ){
			newV=0;
		}
		//jesli aktualna predkosci jest wieksza niz max
		else if(fabs(vel)>Config::getInstance().getRRTMaxVel()){
			double signum=-1;
			if(vel>0)
				signum=1;
			newV=Config::getInstance().getRRTMaxVel()*signum;
		}
		else{
			//std::cout<<"trapez"<<std::endl;
			double s=fabs(targetPosition - currPosition);
			double v=sqrt( (3*dec*pow(vel,2)+2*acc*dec*s)/(acc+dec));

			v=fmin(v,Config::getInstance().getRRTMaxVel());
			double signum=-1;
			if( targetPosition - currPosition > 0 )
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
Vector2D calculateVelocity(const Vector2D &currVel,const  Pose & targetPose){
	Vector2D newVel;
	//std::cout<<"#####################################"<<std::endl;
	//std::cout<<"current robot vel "<<currVel<<std::endl;
	//std::cout<<"curr relative targetPose "<<targetPose<<std::endl;
	//std::cout<<"for X "<<std::endl;
	newVel.x=calculateVelocity(currVel.x, 0,targetPose.get<0>() );
	//std::cout<<"for Y "<<std::endl;
	newVel.y=calculateVelocity(currVel.y, 0,targetPose.get<1>() );
	//std::cout<<"new robot vel "<<newVel<<std::endl;
	//std::cout<<"#####################################"<<std::endl;
	return newVel;
}

Vector2D calculateVelocity(const Vector2D &currVel,const  Pose & currGlobalPose,const  Pose & targetGlobalPose){

	RotationMatrix rmY(currGlobalPose.get<2>());
	//pozycja celu w ukladzie wsp zwiazanych z robotem
	Vector2D targetRelPosition=rmY.Inverse()*(targetGlobalPose.getPosition()-currGlobalPose.getPosition());
	return calculateVelocity( currVel, Pose(targetRelPosition.x,targetRelPosition.y,0));
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
