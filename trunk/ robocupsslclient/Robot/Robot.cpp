#include "Robot.h"
#include "../Logger/Logger.h"
#include "../Config/Config.h"
#include "../VideoServer/Videoserver.h"
#include "../GameState/GameState.h"

#include <boost/foreach.hpp>
#include <boost/bind.hpp>
//#include <boost\math\special_functions\fpclassify.hpp>

#include <gsl/gsl_deriv.h>

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

std::list<Robot::robotID> Robot::getAllRobots(){
	std::list<Robot::robotID> robots;
	robots.push_back( Robot::red0 );
	robots.push_back( Robot::red1 );
	robots.push_back( Robot::red2 );
	robots.push_back( Robot::blue0);
	robots.push_back( Robot::blue1);
	robots.push_back( Robot::blue2);
	return robots;
}

std::list<Robot::robotID> Robot::getBlueTeam(){
	std::list<Robot::robotID> robots;
	robots.push_back( Robot::blue0);
	robots.push_back( Robot::blue1);
	robots.push_back( Robot::blue2);
	return robots;

}

std::list<Robot::robotID> Robot::getRedTeam(){
	std::list<Robot::robotID> robots;
	robots.push_back( Robot::red0 );
	robots.push_back( Robot::red1 );
	robots.push_back( Robot::red2 );
	return robots;
}

Robot::Robot(const std::string robotName_,const std::string posIfaceName) : robotName( robotName_ ), id( Robot::getRobotID(robotName_) ),
		log( getLoggerPtr( robotName_.c_str() ) )
{

	this->posIfaceName=posIfaceName;
	this->v=Vector2D(0,0);
	this->w=0;
	this->time=0;
	this->oldAlfaToCel = 0;

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
	//boost::tuple<double,double,double> currPositions;
	//boost::tuple<double,double,double> newVel=boost::tuple<double,double,double> (v.x,v.y,w);
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
	posIface->data->cmdVelocity.pos.x = v.x;//newVel.get<0>();
	posIface->data->cmdVelocity.pos.y = v.y;//newVel.get<1>();
    posIface->data->cmdVelocity.yaw = w;//newVel.get<2>();

//	posIface->data->cmdVelocity.yaw = (M_PI*newVel.get<2>())/180.0;
//  posIface->data->cmdVelocity.yaw = 0;

    posIface->Unlock();
#endif
	//LOG_TRACE(getLoggerPtr("path"),"set vel       name="<<this->robotName.c_str()<<"\t vx="<<v.x<<"\t vy="<<v.y<<"\t" );

    LOG_TRACE( log, "set vel       name="<<this->robotName.c_str()<<" vx="<<v.x<<" vy="<<v.y<<" w "<<w );
}

void Robot::setGlobalSpeed(const Vector2D & v,const double & w, const double& rot){

	Vector2D speed =v.rotate(-rot);
#ifdef GAZEBO
	posIface->Lock(1);
	posIface->data->cmdEnableMotors = 1;
	posIface->data->cmdVelocity.pos.x = speed.x;
	posIface->data->cmdVelocity.pos.y = speed.y;
    posIface->data->cmdVelocity.yaw = w;
    posIface->Unlock();
#endif
    LOG_TRACE(log,"set vel  "<<" vx="<<speed.x<<" vy="<<speed.y<<" w "<<w );
	//LOG_TRACE(getLoggerPtr("path"),"set vel       name="<<this->robotName.c_str()<<"\t vx="<<speed.x<<"\t vy="<<speed.y<<"\t" );

}

std::pair<Vector2D,double> Robot::getDesiredVel() const
{
	return std::pair<Vector2D,double>(this->v,this->w);
}
std::pair<Vector2D,double> Robot::getRelativeVelocity() const
{
	double vx=0,vy=0,w=0;
#ifdef GAZEBO
	posIface->Lock(1);
	vx = posIface->data->velocity.pos.x;
	vy = posIface->data->velocity.pos.y;
	w = posIface->data->cmdVelocity.yaw;
	//w = 180*w/M_PI;
	posIface->Unlock();
#endif

//	if(robotName.compare(Config::getInstance().getTestModelName())==0){
		//LOG_DEBUG(getLoggerPtr("path"),"robot "<<this->robotName.c_str()<<" from gazebo vx"=%lf\t vy=%lf\t w=%lf,"
		//			,,vx,vy,w);
//		;
//	}

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


bool Robot::disperse( const double dist ){
	//odsuwa robota od przeszkod za pomoca metody pol potencjalowych
	//konstruuje pole ujemne
	LOG_INFO(log,"start  disperse for robot "<<this->robotName<<" minimal distance from obstacle "<<dist );

	//Vector2D Robot::repulsivePotentialField(Vector2D positionCoordinates, std::list< Vector2D > obstacles)
	std::list< Vector2D >obstacles;
	Pose currRobotPose;
	Pose ballPose;
	// bool obstaclesFree = false;
	double distToObstacle = numeric_limits<double>::infinity();

	double lastTime = 0;
	double currTime = 0;

	distToObstacle = numeric_limits<double>::infinity();

	GameStatePtr gameState( new GameState() );
	Videoserver::getInstance().updateGameState( gameState );

	Vector2D goalPose;
	if( strncmp( this->robotName.c_str(), "red", 3 ) == 0 ){
		goalPose = Videoserver::getInstance().getRedGoalMidPosition();
		LOG_INFO(log,"red robot, goal Pose "<<goalPose);
	}else{
		goalPose = Videoserver::getInstance().getBlueGoalMidPosition();
		LOG_INFO(log,"blue robot, goal Pose "<<goalPose);
	}

	Vector2D oldGradient(0.0,0.0);
	Vector2D gradient(0.0,0.0);

	//int cnt=0;
	//odleglosc robota od pilki jest mniejsza niz 30 cm
	do {

		GameStatePtr gameState( new GameState() );
		if( ( currTime = Videoserver::getInstance().updateGameState( gameState ) ) > lastTime  )
		{
			distToObstacle = numeric_limits<double>::infinity();
			lastTime = currTime;
			ballPose = gameState->getBallPos();

			const std::vector<std::string> blueTeam=Config::getInstance().getBlueTeam();

			currRobotPose = gameState->getRobotPos( this->getRobotID( ) );

			Pose nearestObsPose;

			BOOST_FOREACH(std::string modelName,blueTeam){
				if(modelName.compare( this->getRobotName() )!=0){
					obstacles.push_back( gameState->getRobotPos( Robot::getRobotID( modelName) ).getPosition() );

					if( gameState->getRobotPos( Robot::getRobotID( modelName) ).distance( currRobotPose )  <  distToObstacle ){
						distToObstacle =  gameState->getRobotPos( Robot::getRobotID( modelName) ).distance( currRobotPose );
						nearestObsPose = gameState->getRobotPos( Robot::getRobotID( modelName) );
					}

				}
				//else
				//	currRobotPose = gameState->getRobotPos( Robot::getRobotID( modelName) );
			}

			const std::vector<std::string> redTeam=Config::getInstance().getRedTeam();
			BOOST_FOREACH(std::string modelName,redTeam){
				if(modelName.compare( this->getRobotName() )!=0){
					obstacles.push_back( gameState->getRobotPos( Robot::getRobotID(modelName) ).getPosition() );

					if( gameState->getRobotPos( Robot::getRobotID( modelName) ).distance( currRobotPose )  <  distToObstacle ){
						distToObstacle =  gameState->getRobotPos( Robot::getRobotID( modelName) ).distance( currRobotPose );
						nearestObsPose = gameState->getRobotPos( Robot::getRobotID( modelName) );
					}
				}
				//else
				//	currRobotPose = gameState->getRobotPos( Robot::getRobotID( modelName) );
			}

			//Vector2D oldGradient(0.0,0.0);
			//gradient = this->repulsivePotentialField(currRobotPose.getPosition(),goalPo se,obstacles);
			//double granurality = 0.001;
			//gradient = this->navigationFunctionGradient2( currRobotPose.getPosition(),goalPose,obstacles,granurality );

			double val = navigationFunction( currRobotPose.getPosition(), goalPose, obstacles );

			LOG_INFO(log,"navigationFunction in " << currRobotPose.getPosition() << " val="<<val);

			Vector2D g;
			gsl_function F;
			double result, abserr;
			funcParams params;
			params.obstacles = obstacles;
			params.goal = goalPose;
			params.param = currRobotPose.getPosition().y;

			F.function = &navigationFunctionX;
			F.params = &params;

			gsl_deriv_central (&F, 2.0, 1e-8, &result, &abserr);
			g.x = result;

			params.goal = goalPose;
			params.param = currRobotPose.getPosition().x;

			F.function = &navigationFunctionY;
			F.params = &params;

			gsl_deriv_central (&F, 2.0, 1e-8, &result, &abserr);
			g.y = result;

			LOG_INFO(log," gradient from lib dx="<<g.x<<" dy="<<g.y);
			LOG_INFO(log," gradient calculated dx="<<gradient.x<<" dy="<<gradient.y);

			//gradient = gradient * (-1.0);
			gradient = g ;

			//gradient = this->navigationFunctionGradient( currRobotPose.getPosition(),goalPose,obstacles);

			/*
			if( gradient.x * oldGradient.x  < 0.0 ||  gradient.y * oldGradient.y  < 0.0 ){
				cnt++;
			}

			if( cnt == 5 ){
				gradient = this->repulsivePotentialField( currRobotPose.getPosition(), obstacles );
			}
			*/

			oldGradient=gradient;

			//Pose targetGlobalPose = currRobotPose + gradient;
			//Vector2D newSpeed = calculateVelocity(gameState->getRobotGlobalVelocity( this->getRobotID( ) ), currRobotPose , targetGlobalPose);

			obstacles.clear();


			Vector2D velocity = gradient;
			double max;
			if( fabs( velocity.x ) > fabs( velocity.y ) )
				max = fabs( velocity.x );
			else
				max = fabs( velocity.y );

			//double max = fabs( velocity.x ) > fabs( velocity.y ) ? fabs( velocity.x ) : fabs( velocity.y );

			if( max > 0 )
				velocity = velocity * ( 0.5/max );

			LOG_INFO( log,"gradient "<< gradient <<"set speed "<<velocity<<" distTo nearest obs "<<distToObstacle<< " nearestObsPose"<<nearestObsPose << " dist to ball "<<ballPose.distance( currRobotPose ) );


			this->setGlobalSpeed( velocity,0,currRobotPose.get<2>() );

			//this->setGlobalSpeed( newSpeed,0,currRobotPose.get<2>() );
		}

	}while( /*( ballPose.distance( currRobotPose )  < dist ) ||*/  distToObstacle < dist    );

	this->stop();

	LOG_INFO( log,"end  disperse for robot "<<this->robotName );

	return true;
}

Vector2D Robot::repulsivePotentialField( const Vector2D positionCoordinates, const Vector2D goal, std::list< Vector2D > obstacles){

	//double result;

	const double obstacleInfluence = 0.5;//kazda przeszkoda odddzalywuje na 1 metr
	double fieldMagnitude = 0.01;

	std::list< Vector2D >::iterator ii = obstacles.begin();
	assert( obstacles.size() < 7 );
	Vector2D gradient( 0.0, 0.0 );
	Vector2D tmp( 0.0, 0.0 );
	for(; ii != obstacles.end(); ii++ ){
		if( ii->distance(positionCoordinates) > obstacleInfluence){
			tmp = Vector2D(0.0,0.0);
		}
		else{

			tmp.x = fieldMagnitude*( 1.0/( euclideanNorm(ii->x, positionCoordinates.x ) - Config::getInstance().getRRTRobotRadius() ) - ( 1.0/obstacleInfluence ) )  *
					( 1.0/( pow( euclideanNorm(ii->x, positionCoordinates.x ) - Config::getInstance().getRRTRobotRadius(),2)  ) ) *
					(positionCoordinates.x  - ii->x )/ ( euclideanNorm(ii->x, positionCoordinates.x )  - Config::getInstance().getRRTRobotRadius() )  ;

			tmp.y = fieldMagnitude*( 1.0/( euclideanNorm(ii->y, positionCoordinates.y ) - Config::getInstance().getRRTRobotRadius() ) - ( 1.0/obstacleInfluence ) )  *
								( 1.0/( pow( euclideanNorm(ii->y, positionCoordinates.y ) - Config::getInstance().getRRTRobotRadius() ,2)  ) ) *
								(positionCoordinates.y  - ii->y )/ ( euclideanNorm(ii->y, positionCoordinates.y ) - Config::getInstance().getRRTRobotRadius() ) ;

			//tmp.x =( 1.0/( euclideanNorm(ii->x, positionCoordinates.x ) - Config::getInstance().getRRTRobotRadius() ) ) b
			//tmp.y =;
		}
		gradient = gradient + tmp;
	}

	double eps = -10000.0;
	//sila przyciagania
	gradient.x += eps*( positionCoordinates.x -  goal.x);
	gradient.y += eps*( positionCoordinates.y -  goal.y);

	return gradient;
}

Vector2D Robot::repulsivePotentialField( const Vector2D positionCoordinates, std::list< Vector2D > obstacles){

	//double result;

	const double obstacleInfluence = 0.5;//kazda przeszkoda odddzalywuje na 1 metr
	double fieldMagnitude = 0.01;

	std::list< Vector2D >::iterator ii = obstacles.begin();
	assert( obstacles.size() < 7 );
	Vector2D gradient( 0.0, 0.0 );
	Vector2D tmp( 0.0, 0.0 );
	for(; ii != obstacles.end(); ii++ ){
		if( ii->distance(positionCoordinates) > obstacleInfluence){
			tmp = Vector2D(0.0,0.0);
		}
		else{

			tmp.x = fieldMagnitude*( 1.0/( euclideanNorm(ii->x, positionCoordinates.x ) - Config::getInstance().getRRTRobotRadius() ) - ( 1.0/obstacleInfluence ) )  *
					( 1.0/( pow( euclideanNorm(ii->x, positionCoordinates.x ) - Config::getInstance().getRRTRobotRadius(),2)  ) ) *
					(positionCoordinates.x  - ii->x )/ ( euclideanNorm(ii->x, positionCoordinates.x )  - Config::getInstance().getRRTRobotRadius() )  ;

			tmp.y = fieldMagnitude*( 1.0/( euclideanNorm(ii->y, positionCoordinates.y ) - Config::getInstance().getRRTRobotRadius() ) - ( 1.0/obstacleInfluence ) )  *
								( 1.0/( pow( euclideanNorm(ii->y, positionCoordinates.y ) - Config::getInstance().getRRTRobotRadius() ,2)  ) ) *
								(positionCoordinates.y  - ii->y )/ ( euclideanNorm(ii->y, positionCoordinates.y ) - Config::getInstance().getRRTRobotRadius() ) ;

			//tmp.x =( 1.0/( euclideanNorm(ii->x, positionCoordinates.x ) - Config::getInstance().getRRTRobotRadius() ) ) b
			//tmp.y =;
		}
		gradient = gradient + tmp;
	}

	//double eps = -100;
	//sila przyciagania
	//gradient.x += eps*( positionCoordinates.x -  goal.x);
	//gradient.y += eps*( positionCoordinates.y -  goal.y);

	return gradient;
}

double betai( Vector2D q, Vector2D qi, double radious){
	return pow( euclideanNorm(q,qi),2) - pow(radious,2);
}

double gradientBetai(double q, double qi){
	return 2*(q-qi);
}

double navigationFunctionX( double x, void * p){
	funcParams* params = reinterpret_cast< funcParams* >( p );

	Vector2D positionCoordinates(x,params->param);

	double beta = -1.0*pow( euclideanNorm( positionCoordinates, Config::getInstance().field.FIELD_MIDDLE_VECTOR),2 ) + pow( Config::getInstance().field.FIELD_LENGTH/2.0 , 2) ;

	std::list< Vector2D >::iterator ii = params->obstacles.begin();
	for( ;ii!=params->obstacles.end();ii++ ){

		beta*=( pow( euclideanNorm( positionCoordinates, params->goal),2 ) - pow( 2.0*Config::getInstance().getRRTRobotRadius() , 2) );
	}
	assert( beta >= 0);

	double lambda = 1;
	double kappa = 10;

	double fi = pow( euclideanNorm( positionCoordinates, params->goal),2 )/ pow( lambda*beta + pow( euclideanNorm( positionCoordinates, params->goal),2.0*kappa ), 1.0/kappa );

	return 1.0*fi;
}

double navigationFunctionY( double y, void * p){
	funcParams* params = reinterpret_cast< funcParams* >( p );

	Vector2D positionCoordinates(params->param,y);

	double beta = -1.0*pow( euclideanNorm( positionCoordinates, Config::getInstance().field.FIELD_MIDDLE_VECTOR),2 ) + pow( Config::getInstance().field.FIELD_LENGTH/2.0 , 2) ;

	std::list< Vector2D >::iterator ii = params->obstacles.begin();
	for( ;ii!=params->obstacles.end();ii++ ){

		beta*=( pow( euclideanNorm( positionCoordinates, params->goal),2 ) - pow( 2.0*Config::getInstance().getRobotMainCylinderRadious() , 2) );
	}
	assert( beta >= 0);

	double lambda = 1;
	double kappa = 10;

	double fi = pow( euclideanNorm( positionCoordinates, params->goal),2 )/ pow( lambda*beta + pow( euclideanNorm( positionCoordinates, params->goal),2.0*kappa ), 1.0/kappa );

	return 1.0*fi;
}


double Robot::navigationFunction( const Vector2D positionCoordinates, const Vector2D goal, std::list< Vector2D > obstacles){

	double beta = -1.0*pow( euclideanNorm( positionCoordinates, Config::getInstance().field.FIELD_MIDDLE_VECTOR),2 ) + pow( Config::getInstance().field.FIELD_LENGTH/2.0 , 2) ;

	std::list< Vector2D >::iterator ii = obstacles.begin();
	for( ;ii!=obstacles.end();ii++ ){

		beta*=( pow( euclideanNorm( positionCoordinates, goal),2 ) - pow( Config::getInstance().getRRTRobotRadius() , 2) );
	}
	assert( beta >= 0);

	double lambda = 1;
	double kappa = 10;

	double fi = pow( euclideanNorm( positionCoordinates, goal),2 )/ pow( lambda*beta + pow( euclideanNorm( positionCoordinates, goal),2.0*kappa ), 1.0/kappa );

	return fi;
}

Vector2D Robot::navigationFunctionGradient2( const Vector2D positionCoordinates, const Vector2D goal, std::list< Vector2D > obstacles, double granularity){

	double minDX = numeric_limits<double>::infinity();
	double minDY = numeric_limits<double>::infinity();

	double X=0;
	double Y=0;
	double minFi = numeric_limits<double>::infinity();

	double eps = 0.001;

	Vector2D v;

	v.x = ( navigationFunction( Vector2D(positionCoordinates.x+ eps,positionCoordinates.y), goal, obstacles) -
			navigationFunction( Vector2D( positionCoordinates.x - eps ,positionCoordinates.y ), goal, obstacles) ) /2.0*eps ;

	v.y = ( navigationFunction( Vector2D( positionCoordinates.x, positionCoordinates.y + eps ), goal, obstacles) -
			navigationFunction( Vector2D(positionCoordinates.x ,positionCoordinates.y - eps ), goal, obstacles) ) /2.0*eps ;
	/*
	//wyszukaj kierunek najwiekszego spadku
	for(double x = positionCoordinates.x - 1.0 ; x < positionCoordinates.x + 1.0; x += granularity ){
		for(double y = positionCoordinates.y - 1.0 ; y < positionCoordinates.y + 1.0; y += granularity ){

			double tmp = ( navigationFunction( Vector2D(x+ eps,y), goal, obstacles) - navigationFunction( Vector2D(x - eps ,y), goal, obstacles) ) /2.0*eps ;

			if( tmp < minFi  ){
				X = x;
				Y = y;
				tmp = minFi;
			}
		}

	}*/

	return v;
}

Vector2D Robot::navigationFunctionGradient( const Vector2D positionCoordinates, const Vector2D goal, std::list< Vector2D > obstacles){

	std::list< Vector2D >::iterator ii = obstacles.begin();
	assert( obstacles.size() < 7 );

	double beta = 0;

	double gradientBetaX = 0;

	double gradientBetaY = 0;

	//unsigned int obsNr=0;

	beta = ( -1.0*pow( euclideanNorm( positionCoordinates, Config::getInstance().field.FIELD_MIDDLE_VECTOR),2 ) + pow( Config::getInstance().field.FIELD_LENGTH/2.0 , 2) );

	assert( beta >= 0.0 );

	for( unsigned int i =0; ii!= obstacles.end(); i++ ){


		double temp =1;
		if(i >0 )
			temp = ( -1.0*pow( euclideanNorm( positionCoordinates, Config::getInstance().field.FIELD_MIDDLE_VECTOR),2 ) + pow( Config::getInstance().field.FIELD_LENGTH/2.0 , 2) );
			//temp=betai( positionCoordinates,Config::getInstance().field.FIELD_MIDDLE_VECTOR,  Config::getInstance().field.FIELD_LENGTH/2.0 );

		for( unsigned int j=0; j < obstacles.size();j++ ){
			if( j!=i ){
				/*if(i==0){
					temp = betai(positionCoordinates, *ii, 2.0*Config::getInstance().getRRTRobotRadius() );

					beta = ( pow( euclideanNorm( positionCoordinates, *ii),2 ) - pow( 2.0*Config::getInstance().getRRTRobotRadius() , 2) );
				}*/
				//else{
					temp *= betai(positionCoordinates, *ii, 1.5*Config::getInstance().getRRTRobotRadius() );

					assert( boost::math::isnormal( temp ) );
					//beta *= ( pow( euclideanNorm( positionCoordinates, *ii),2 ) - pow( 2.0*Config::getInstance().getRRTRobotRadius() , 2) );
				//}
			}
		}
		if(i==0)
			gradientBetaX += (-2.0*( positionCoordinates.x -  Config::getInstance().field.FIELD_MIDDLE_VECTOR.x ) )  * temp;
		else
			gradientBetaX +=gradientBetai(positionCoordinates.x, ii->x) * temp;

		assert( boost::math::isnormal(gradientBetaX) );

		if(i==0)
			gradientBetaY += (-2.0*(positionCoordinates.y -  Config::getInstance().field.FIELD_MIDDLE_VECTOR.y) )  * temp;
		else{
			gradientBetaY +=gradientBetai(positionCoordinates.y, ii->y) * temp;
			ii++;
		}
		assert( boost::math::isnormal(gradientBetaY) );

		//TODO:
		//sprawdzic jaki jest faktyczny promien przeszkody tj innego robota
		beta *= ( pow( euclideanNorm( positionCoordinates, *ii),2.0 ) - pow( 1.5*Config::getInstance().getRRTRobotRadius() , 2.0 ) );

		assert( beta >= 0.0 );

		assert( boost::math::isnormal( beta ) );

		//obsNr++;
	}

	double gradientX = 0;

	double gradientY = 0;

	double derivativeDX = (positionCoordinates.x - goal.x)/euclideanNorm( positionCoordinates,goal);

	double derivativeDY = (positionCoordinates.y - goal.y)/euclideanNorm( positionCoordinates,goal);

	double lambda = 10.0;
	double d = euclideanNorm( positionCoordinates,goal);
	double kappa = 2.0;

	double a, b, c , e;

	assert( boost::math::isnormal( derivativeDX ) );

	assert( boost::math::isnormal( d ) );

	a = 2.0*d*derivativeDX*( pow( lambda*beta + pow(d,2.0*kappa), 2.0/kappa) );
	if( !boost::math::isnormal( a ) ){
		std::cout<<"2.0*d*derivativeDX"<<2.0*d*derivativeDX<<" pow( lambda*beta + pow(d,2.0*kappa), 2.0/kappa) "<<pow( lambda*beta + pow(d,2.0*kappa), 2.0/kappa)<<std::endl;
	}
	assert( boost::math::isnormal( a ) );

	assert( beta >= 0.0 );

	b = (1.0/kappa)*pow( lambda*beta + pow(d,2.0*kappa), 1.0/kappa -1.0);
	assert( boost::math::isnormal( b ) );

	c = (lambda*gradientBetaX + 2.0*kappa*pow(d,2.0*kappa-1.0) )*derivativeDX;
	assert( boost::math::isnormal( c ) );

	e = pow( (lambda*beta + pow(d,2.0*kappa)), 2.0/kappa );
	assert( boost::math::isnormal( d ) );

	gradientX =1.0*( a - b*c )/e;

	a = 2.0*d*derivativeDY*( pow( lambda*beta + pow(d,2.0*kappa), 2.0/kappa) );
	assert( boost::math::isnormal( a ) );

	b = (1.0/kappa)*pow( lambda*beta + pow(d,2.0*kappa), 1.0/kappa -1.0);
	assert( boost::math::isnormal( b ) );

	c = (lambda*gradientBetaY + 2.0*kappa*pow(d,2.0*kappa-1.0) )*derivativeDY;
	assert( boost::math::isnormal( c ) );

	e = pow( (lambda*beta + pow(d,2.0*kappa)), 2.0/kappa );
	assert( boost::math::isnormal( d ) );

	gradientY = 1.0*( a - b*c )/e;

	/*
	gradientY =-1.0*( 2.0*d*derivativeDY*( pow( lambda*beta + pow(d,2*kappa), 2.0/kappa) ) - (1.0/kappa)*pow( lambda*beta + pow(d,2.0*kappa), 1.0/kappa -1.0)*
			(lambda*gradientBetaY + 2.0*kappa*pow(d,2.0*kappa-1) )*derivativeDY )/pow( (lambda*beta + pow(d,2.0*kappa)), 2.0/kappa );
	 */

	return Vector2D(gradientX,gradientY);
}


bool Robot::isRed(Robot::robotID id_){
	if(id_ < 0){
		return true;
	}
	else
		return false;
}
bool Robot::isBlue(Robot::robotID id_){
	if(id_ > 0){
		return true;
	}
	else
		return false;
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
	//bool finish=false;
	double err=1;
	while( err > 0.01 ){
		#ifdef GAZEBO
			posIface->Lock(1);
			err=( pow( posIface->data->velocity.pos.x, 2 ) + pow( posIface->data->velocity.pos.y, 2 ) );
			posIface->Unlock();
		#endif
			usleep(10000);
	}

}

double Robot::calculateAngularVel(const  Pose & globalRobotPose, const  Vector2D & globalTargetPosition){
	Pose p( globalTargetPosition,0.0 );
	return calculateAngularVel( globalRobotPose, p);
}
double Robot::calculateAngularVel(const  Pose & globalRobotPose, const  Pose & globalTargetPose){

	//Pose globalTargetPosition = globalTargetPosition_*100;
	//obrot jaki trzeba było wykonac w poprzednim kroku
	//static double oldAlfaToCel;

    double Ker=0.5;
    double Ko=20;
    //double currGlobalRobotRot=gameState.getRobotPos( robotID ).get<2>();
    double currGlobalRobotRot = globalRobotPose.get<2>();
    //pozycja robota w ukladzie wsp zw z plansza
    //Pose currRobotPose=gameState.getRobotPos( robotID );

    // ten kawalek kodu wyznacza kat o jaki robot musi sie obrocic zeby byc skierowanym na cel
    RotationMatrix rm0(0);
    Pose reltargetPose_ = globalTargetPose.transform( globalRobotPose.getPosition(),rm0 );
    Pose reltargetPose = reltargetPose_*100;
    double rotacjaDocelowa=-atan2(reltargetPose.get<0>(),reltargetPose.get<1>()) ;


    assert( fabs(rotacjaDocelowa) < M_PI);

    //double rotacjaDocelowa=-atan2(reltargetPose.get<1>(),reltargetPose.get<0>()) ;// (M_PI/2);
    //macierz obrotu os OY na wprost robota
    //RotationMatrix rmY(currGlobalRobotRot);
    //macierz obrotu os OY nw wprost robota
    //RotationMatrix rmY(-M_PI/2);
    //pozycja celu w ukladzie wsp zwiazanych z robotem
    //Pose reltargetPose=globalTargetPosition.transform(currRobotPose.getPosition(),rmY);
    //targetPosition=rmX.Inverse()*(goToPosition-currentPosition);

    //obrot jaki trzeba wykonac w biezacym kroku
    double currAlfaToCel = convertAnglePI( rotacjaDocelowa - currGlobalRobotRot );///convertAnglePI( currGlobalRobotRot - rotacjaDocelowa  );


    if( fabs( currAlfaToCel )  < 0.01 )
    	return 0;
    //kierunek obrotu
    //iotacjaDocelowa * currGlobalRobotRot < 0 ){
    //	currAlfaToCel=currAlfaToCel*(-1);
    //}

    //double currTetaCel=atan( (-reltargetPose.get<1>()) / (reltargetPose.get<0>()));
    // double angularVel= Ko*(blad rotacji) + Ker(obrotdo celu jaki trzeba bylo wykonac w poprzednm kroku - obrot do celu jaki trzeba wykonac w tymkroku)

    //double angularVel=Ko*(rotacjaDocelowa-currGlobalRobotRot)+ Ker*(oldAlfaToCel - currAlfaToCel);
    double angularVel=Ko*(  currAlfaToCel  )+ Ker*( convertAnglePI( this->oldAlfaToCel - currAlfaToCel) );

    double w = fabs(angularVel) > M_PI/2 ? M_PI/2 * sgn(angularVel) : angularVel;

//    if( this->oldAlfaToCel*currAlfaToCel < 0 ){
//    	if( pow( this->oldAlfaToCel - currAlfaToCel ,2 ) )
//    		w=-1*w;
//    }

    this->oldAlfaToCel=currAlfaToCel;


//    LOG_INFO( log,"relative target pose "<<reltargetPose<<std::endl );
    LOG_TRACE(log,"potrzebny obrot do celu "<<currAlfaToCel<<" rotacjaDocelowa "<<rotacjaDocelowa<<" currGlobalRobotRot "<<currGlobalRobotRot<<" calculate angular vel w="<< w);

    return  w;
}

/*
double Robot::calculateAngularVel(GameState & gameState,Robot::robotID robotID, Pose targetPosition){
    static double oldTetaCel;
    //double rotacjaDocelowa=atan2(targetPosition.get<0>(),targetPosition.get<2>());
    double rotacjaDocelowa=atan2(targetPosition.get<1>(),targetPosition.get<0>());
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

    double w = fabs(angularVel) > 4*M_PI ? 4*M_PI * sgn(angularVel) : angularVel;
    LOG_INFO(log,"rotacjaDocelowa "<<currTetaCel<<" calculate angular vel w="<< w);
    return  w;
}
*/
Robot::~Robot()
{
	this->posIface->Close();
	delete this->posIface;
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
	/*Uwaga
	 *
	 * przy duzych odleglosiach fabs( targetPosition-currPosition )* vel jest < 0.01
	 *
	 * co wiecej gdy zadamy robotowi max predkosc w jednym kierunku,
	 * to zawsze ma sladowa w kierunku prostopadlym
	 *
	 */
		//if( vel*( targetPosition-currPosition ) < -0.01){
		//	newV=0;
		//}
		//jesli poruszajac sie z aktualna predkoscia przekroczymy cel
		/*else*/ if( fabs(targetPosition-currPosition)<= pow(vel,2)/(2*dec)  ){
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

			//v=fmin(v,Config::getInstance().getRRTMaxVel());
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
/*
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
*/
/*
Vector2D calculateVelocity(const Vector2D &currVel,const  Pose & currGlobalPose,const  Pose & targetGlobalPose){

	RotationMatrix rmY( currGlobalPose.get<2>() );
	//pozycja celu w ukladzie wsp zwiazanych z robotem
	Vector2D targetRelPosition=rmY.Inverse()*(targetGlobalPose.getPosition()-currGlobalPose.getPosition());

	Vector2D newVel;
	newVel.x=calculateVelocity(currVel.x, 0,targetRelPosition.get<0>() );
	newVel.y=calculateVelocity(currVel.y, 0,targetRelPosition.get<1>() );

	return newVel;//calculateVelocity( currVel, Pose(targetRelPosition.x,targetRelPosition.y,0));
}
*/

Vector2D calculateVelocity(const Vector2D &currVel,const  Pose & currGlobalPose,const  Pose & targetGlobalPose){

	//RotationMatrix rmY( currGlobalPose.get<2>() );
	//pozycja celu w ukladzie wsp zwiazanych z robotem
	//Vector2D targetRelPosition=rmY.Inverse()*(targetGlobalPose.getPosition()-currGlobalPose.getPosition());

	Vector2D newVel;
	newVel.x=calculateVelocity(currVel.x, currGlobalPose.get<0>(),targetGlobalPose.get<0>() );
	newVel.y=calculateVelocity(currVel.y, currGlobalPose.get<1>(),targetGlobalPose.get<1>() );

	double scale = fabs(newVel.x) > fabs(newVel.y) ? fabs(newVel.x) : fabs(newVel.y) ;
	if( scale > Config::getInstance().getRRTMaxVel() )
		scale=Config::getInstance().getRRTMaxVel()/scale;
	//Vector2D v=newVel*scale;
	return newVel*scale;

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
