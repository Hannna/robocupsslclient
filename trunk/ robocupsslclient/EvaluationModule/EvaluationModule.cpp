#include "EvaluationModule.h"
#include "EvaluationTypes.h"
#include "../Lock/Lock.h"
#include "../additional.h"
#include "../Config/Config.h"
#include "../Set/Set.h"
#include "../Robot/Robot.h"
#include "../Exceptions/SimulationException.h"

#include "SimAnnealing2.h"
#include "FindBestPointToShoot.h"

#include <math.h>
#include <limits>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>


EvaluationModule * EvaluationModule::ptr=NULL;
Mutex EvaluationModule::mutex;


 EvaluationModule& EvaluationModule::getInstance(){
    if(ptr)
        return *ptr;
    else{
        LockGuard m(mutex);
        if(ptr)
            return *ptr;
        else{
            ptr = new EvaluationModule();
            return *ptr;
        }

    }
}

EvaluationModule::EvaluationModule():video(Videoserver::getInstance()), appConfig(Config::getInstance()),
        log(getLoggerPtr ("app_debug"))
{

}

score EvaluationModule::aimAtTeamMate(Robot::robotID shootingRobotID, Robot::robotID goalRobotID, double * rotationToTarget){
	GameStatePtr gameState( new GameState() );

	if( video.updateGameState( gameState ) < 0){
		std::ostringstream s;
		s<<__FILE__<<":"<<__LINE__;
		throw SimulationException( s.str() );
	}

	score score_ = 0;

	Pose shootingRobotPose=gameState->getRobotPos( shootingRobotID );
	Pose goalRobotPose=gameState->getRobotPos( goalRobotID );

	//obliczam wspolrzedna srodka robota do ktorego kierowane jest podanie w ukladzie wsp zw z robotem strzelajacym
	RotationMatrix rm( shootingRobotPose.get<2>() );

	Pose recvPassRelPos = goalRobotPose.transform( shootingRobotPose.getPosition() , rm );

	if( rotationToTarget )
		*rotationToTarget = calculateProperAngleToTarget( shootingRobotPose, goalRobotPose );



	//jesli wsp x jest bliska 0 i roboty maja przeciwna rotace to mozna zrealizowac podanie
	if( pow(recvPassRelPos.get<0>(),2) < 0.05*recvPassRelPos.get<1>() ){
		//roboty maja rotacje odpowiednia do siebie gdy roznica jest rowna -M_PI
		//double deviation = pow(  fabs( shootingRobotPose.get<2>() - goalRobotPose.get<2>() )  - M_PI, 2 ) ;
		double deviation = fabs( shootingRobotPose.get<2>() - goalRobotPose.get<2>() )  - M_PI ;

		double threshold = 0.1*recvPassRelPos.get<1>();
		if( deviation < threshold   ){// 4 stopnie
			LOG_DEBUG( log," mozna podac deviation="<<deviation<<" threshold "<<threshold );
			//std::cout<<" mozna podac deviation="<<deviation<<std::endl;
			score_ = 1.0 - deviation;
		}
		else{
			LOG_INFO(log, "deviation "<<deviation );
			LOG_INFO(log, "shootingRobotPose.get<2>() "<<shootingRobotPose.get<2>()<<" goalRobotPose.get<2>() "<< goalRobotPose.get<2>() );
		}
	}
	else{
		if(rotationToTarget){
			LOG_INFO(log, "!!!!!!!!! bad recvPassRelPos "<<recvPassRelPos<<" rotationToTarget "<<*rotationToTarget<<" currRotation "<<shootingRobotPose.get<2>() );
		}
		else{
			LOG_INFO(log, "!!!!!!!!! bad recvPassRelPos "<<recvPassRelPos<<" currRotation "<<shootingRobotPose.get<2>() )
		}
	}

	return score_;
}

bool EvaluationModule::checkAngleToPass(Vector2D targetPosition, Pose currRobotPosition, double & angleToTarget) const{

	angleToTarget = calculateAngleToTarget( currRobotPosition, Pose(targetPosition.x,targetPosition.y,0) );
	double threshold = 0.017 * currRobotPosition.getPosition().distance(targetPosition);
	//LOG_INFO(this->log,"set threshold to "<<threshold);

	if( fabs( angleToTarget ) < threshold ){//1stopien
		return true;
	}
	return false;
}


/*
std::pair<double, double> EvaluationModule::aimAtTeamMate(Robot::robotID shootingRobotID, Robot::robotID goalRobotID){
	GameStatePtr gameState( new GameState() );
	video.updateGameState( gameState );
	Pose shootingRobotPose=gameState->getRobotPos( shootingRobotID );
	Pose goalRobotPose=gameState->getRobotPos( goalRobotID );

	//Obliczam kat do robota bedacego celem
	//wersor osi ox
	Vector2D ox(0,1);
	double alfa1;
	double alfa2;
	double dist;

	Pose goalPose = gameState->getRobotPos(goalRobotID);

	RotationMatrix rm( goalRobotPose.get<2>() );
	//wspolrzedna kranca driblerra w ukladzie wspolrzednych zwiazanym z robotem
	Pose p1( -0.035,0.06, 0 );

	Pose p1global = p1.transform( goalRobotPose.getPosition(), rm );
	Pose p1l = p1global.translation( shootingRobotPose.getPosition() );

	Pose p2(0.035,0.06, 0);
	Pose p2global = p2.transform( goalRobotPose.getPosition(), rm );

	Pose p2r = p2global.translation( shootingRobotPose.getPosition() );

	alfa1 = p1l.getPosition().angleTo(ox);
	alfa2 = p2r.getPosition().angleTo(ox);

	dist = shootingRobotPose.getPosition().distance( goalRobotPose.getPosition() );
	//dist = appConfig.field.TOP_GOAL_MID_POSITION.distance( robotPose.getPosition() );


    LOG_DEBUG( log," open angle to the red top goal min "<<alfa1<<" max "<<alfa2 );


	double alfamin = alfa1 < alfa2 ? alfa1 : alfa2;
	double alfamax = alfa1 > alfa2 ? alfa1 : alfa2;

	Set maxOpenAngle( alfamin, alfamax, dist );

	Set tmpAng(maxOpenAngle);

	std::list< Set > angles;
	angles.push_back(maxOpenAngle);

	std::vector<Pose> positions=gameState->getEnemyRobotsPos( shootingRobotID );
	std::vector<Pose>::iterator ii=positions.begin();
	for(ii = positions.begin(); ii!=positions.end(); ii++){
		Set ang = findObstacleCoverAngles( shootingRobotPose,*ii);

		if( ang.d > 0 ){
			addToList( ang , angles);
		}
	}

	//znajdz najszerszy przedzial w kolekcji angles
	std::list<Set>::iterator iii=
		std::max_element(angles.begin(),angles.end(),
			  boost::bind( &Set::width,_1) );


	return std::pair<double,double>( (*iii).angmin, (*iii).angmax );

}
*/

BallState::ballState EvaluationModule::getBallState(Robot::robotID id, bool * iAmCloserToBall ){

	GameStatePtr gameState( new GameState() );

	if( video.updateGameState( gameState ) < 0){
		std::ostringstream s;
		s<<__FILE__<<":"<<__LINE__;
		throw SimulationException( s.str() );
	}

    const std::vector<std::string> blueTeam=Config::getInstance().getBlueTeam();
    BallState::ballState bs=BallState::free;
    BallState::ballState ballState_=BallState::free;

    Pose p;
    Pose ballPose = gameState->getBallPos();

    //pobierz ostatni stan z servera wideo,
    //wazne w przypadku autu badz zdobycia bramki
    ballState_ = gameState->getBallState();//getBallState( ballPose.getPosition() );
    if( ballState_ != BallState::free){
    	//LOG_FATAL( log,"Ball state is  "<<ballState_ );
    	LOG_FATAL( log,"Ball 123position is  "<<ballPose<<" ballState "<<ballState_ );
    	return ballState_;
    }


    if( Robot::isBlue( id ) ){
    	bs = BallState::occupied_our;
    }
    else
    	bs = BallState::occupied_theirs;

    double distanceToBall;
    double angleToBall;

	//dla kazdego robota z naszej druzyny sprawdz czy nie jest posiadaczem pilki
	//dla kazdego robota z druzyny przeciwnej sprawdz czy nie jest posiadaczem pilki

    //odleglosc najblizszego robota do pilki
    double shortestDistance = numeric_limits<double>::max( );
    Robot::robotID closerToBallRobotID;

    Robot::robotID robotID;
	BOOST_FOREACH(std::string modelName,blueTeam){
		robotID = Robot::getRobotID( modelName);
		p = gameState->getRobotPos( robotID );

		//if( p.distance(ballPose) < ( Config::getInstance().getRobotMainCylinderRadious() + 0.04 ) ){
		if( this->isRobotOwnedBall( Robot::getRobotID( modelName ), gameState , distanceToBall, angleToBall ) ){
			ballState_ = bs;
			if( Robot::getRobotID( modelName) == id )
				return BallState::mine;
			else
				return ballState_;
		}
		if( distanceToBall < shortestDistance ){
			shortestDistance=distanceToBall;
			closerToBallRobotID = robotID;
		}

	}

	if(iAmCloserToBall){
		*iAmCloserToBall = false;
		if( closerToBallRobotID == id )
			*iAmCloserToBall = true;
	}


    if( Robot::isBlue(id) ){
    	bs = BallState::occupied_theirs;
    	//this->ballState_ = EvaluationModule::occupied_theirs;
    }
    else{
    	bs = BallState::occupied_our;
    	//this->ballState_ = EvaluationModule::occupied_our;
    }
	const std::vector<std::string> redTeam=Config::getInstance().getRedTeam();
	BOOST_FOREACH(std::string modelName,redTeam){
		p = gameState->getRobotPos( Robot::getRobotID( modelName ) );

		//if( p.distance(ballPose) < ( Config::getInstance().getRobotMainCylinderRadious() + 0.04 ) ){
		if( this->isRobotOwnedBall( Robot::getRobotID( modelName ), gameState , distanceToBall, angleToBall ) ){
			//this->ballSate=ballState;
			ballState_ = bs;
			LOG_DEBUG(log,"Robot have got Ball p.distance(ballPose) "<< p.distance(ballPose) <<" robot name"<<modelName);

			if( Robot::getRobotID( modelName) == id )
				return BallState::mine;
			else
				return ballState_;

			return ballState_;
		}
		else{
			;//LOG_DEBUG(log,"p.distance(ballPose) "<< p.distance(ballPose) <<" robot name"<<modelName);
		}

	}

	//sprawdz czy predkosc pilki nie kieruje jej do bramki
	Vector2D currBallPosition = gameState->getBallPos().getPosition();
	Vector2D ballSpeed = gameState->getBallGlobalVelocity();
	Vector2D predBallPosition = currBallPosition+ballSpeed*10.0;

	if( Config::getInstance().field.BOTTOM_GOAL_MID_POSITION.distance(predBallPosition) <
			Config::getInstance().field.BOTTOM_GOAL_MID_POSITION.distance(currBallPosition) ){
		bs=BallState::go_to_goal;
	}
	//bs = getBallState(ballPosition);
	return ballState_;
}

BallState::ballState EvaluationModule::getBallState(Vector2D ballPosition){
	BallState::ballState bs;
	//sprawdz czy pilka jest w obrebie boiska
		if( ( ballPosition.x >= ( Config::getInstance().field.FIELD_BOTTOM_LEFT_CORNER.x + Config::getInstance().field.FIELD_MARIGIN ) ) &&
				( ballPosition.x  <= ( Config::getInstance().field.FIELD_TOP_RIGHT_CORNER.x - Config::getInstance().field.FIELD_MARIGIN)  ) ){
			if( ( ballPosition.y <= ( Config::getInstance().field.FIELD_TOP_RIGHT_CORNER.y - Config::getInstance().field.FIELD_MARIGIN  ) ) &&
					( ballPosition.y  >= ( Config::getInstance().field.FIELD_BOTTOM_LEFT_CORNER.y + Config::getInstance().field.FIELD_MARIGIN ) ) ){

					bs = BallState::free;
					ballState_ = bs;
					return ballState_;
			}
		}


		//sprawdz czy nie jest w  dolnej bramce
		if( ballPosition.y  <= ( Config::getInstance().field.BOTTOM_GOAL_MID_POSITION.y ) ) {
			if( ( ballPosition.x >= Config::getInstance().field.BOTTOM_GOAL_LEFT_CORNER.x  ) &&
					( ballPosition.x  <= Config::getInstance().field.BOTTOM_GOAL_RIGHT_CORNER.x ) ){

					ballState_ = BallState::in_goal;
					return BallState::in_goal;
			}
		}

		//sprawdz czy nie jest w  gornej bramce
		if( ballPosition.y  >= ( Config::getInstance().field.FIELD_TOP_RIGHT_CORNER.y - Config::getInstance().field.FIELD_MARIGIN ) ) {
			if( ( ballPosition.x >= Config::getInstance().field.TOP_GOAL_LEFT_CORNER.x  ) &&
					( ballPosition.x  <= Config::getInstance().field.TOP_GOAL_RIGHT_CORNER.x  ) ){

					ballState_ = BallState::in_goal;
					return BallState::in_goal;
			}
		}


		//pilka jest na aucie
		if( ballState_ != BallState::out){
			ballState_ = BallState::out;
			this->positionForThrowIn = ballPosition;
			//LOG_INFO( log,"!!!!!!!!!!!!!this->positionForThrowIn "<<this->positionForThrowIn);
		}
		return BallState::out;
}
/*@brief znajduje najbardziej atrakcyjny punkt na planszy
*
* implementacja naiwna, najbardziej atrakcyjny jest srodek planszy
*/
/*
Pose EvaluationModule::findBestDribbleTarget( const std::string robotName,Robot::robotID rid ){

	GameStatePtr currGameState(new GameState());

    if( video.updateGameState(currGameState) < 0){
		std::ostringstream s;
		s<<__FILE__<<":"<<__LINE__;
    	throw SimulationException( s.str() );
    }
	//SimAnnealing2 sim(currGameState, robotName,rid);
    //std::pair<Vector2D, double> s = sim.simAnnnealing2();
    const int distance = 1000;
    FindBestPointToShoot f(Config::getInstance().field.FIELD_MIDDLE_POSE.getPosition(), distance,
    		currGameState,robotName,rid);
    std::pair<Vector2D, double> s =f.bestTarget();

    LOG_INFO( log," FindBestPointToShoot return  "<<s.first );
    //exit(0);
    return Pose(s.first,s.second);//appConfig.field.FIELD_MIDDLE_POSE;

}
*/
Pose EvaluationModule::findBestDribbleTarget( Vector2D centerPoint,const std::string robotName,Robot::robotID rid ){

	GameStatePtr currGameState(new GameState());

    if( video.updateGameState(currGameState) < 0){
		std::ostringstream s;
		s<<__FILE__<<":"<<__LINE__;
    	throw SimulationException( s.str() );
    }
	//SimAnnealing2 sim(currGameState, robotName,rid);
    //std::pair<Vector2D, double> s = sim.simAnnnealing2();
    const int distance = 1000;
    FindBestPointToShoot f(centerPoint, distance,
    		currGameState,robotName,rid);
    std::pair<Vector2D, double> s =f.bestTarget();

    LOG_INFO( log," FindBestPointToShoot return  "<<s.first );
    //exit(0);
    return Pose(s.first,s.second);//appConfig.field.FIELD_MIDDLE_POSE;

}

/*
*@brief sprawdza czy robot jest w posiadaniu pilku
*
*@ret true jesli robot jest w posiadaniu pilki
*/
/*
bool EvaluationModule::haveBall_1(const Robot & robot){
    GameStatePtr currGameState(new GameState());

    if( video.updateGameState(currGameState) < 0){
		std::ostringstream s;
		s<<__FILE__<<":"<<__LINE__;
    	throw SimulationException( s.str() );
    }

    Pose robotPose=currGameState->getRobotPos( robot.getRobotID() );
    Pose ballPose=currGameState->getBallPos();


    RotationMatrix rmY( robotPose.get<2>() );
    //pozycja celu w ukladzie wsp zwiazanych z robotem
    Pose ballRelPose=ballPose.transform(robotPose.getPosition(),rmY);

    //rotacja do celu
    double toBallRot=atan2( (ballRelPose.get<1>()) , (ballRelPose.get<0>()));

    return fabs(toBallRot) < ROTATION_PRECISION ? true : false ;
}
*/

bool EvaluationModule::isRobotOwnedBall(const Robot * robot){
	return isRobotOwnedBall( *robot );
}

bool EvaluationModule::isRobotOwnedBall(const Robot::robotID & robotId){
	double distanceToBall;
	double angleToBall;
    GameStatePtr currGameState(new GameState());
    currGameState->setSimTime( video.getCurrentSimTime() );

	if( video.updateGameState(currGameState) < 0 ){
		std::ostringstream s;
		s<<__FILE__<<":"<<__LINE__;
    	throw SimulationException( s.str() );
    }

	return isRobotOwnedBall( robotId, currGameState, distanceToBall, angleToBall);
}

bool EvaluationModule::isRobotOwnedBall(const Robot & robot){

    GameStatePtr currGameState(new GameState());

    if( video.updateGameState(currGameState) < 0 ){
		std::ostringstream s;
		s<<__FILE__<<":"<<__LINE__;
    	throw SimulationException( s.str() );
    }
    double distanceToBall;
    double angleToBall;
    return isRobotOwnedBall(robot, currGameState,distanceToBall,angleToBall );
}

bool EvaluationModule::isRobotOwnedBall(const Robot & robot, const GameStatePtr& currGameState, double& distanceToBall, double& angleToBall){

	return isRobotOwnedBall( robot.getRobotID( ), currGameState, distanceToBall, angleToBall);
}

bool EvaluationModule::isRobotOwnedBall(const Robot::robotID & robotID, const GameStatePtr& currGameState,double& distanceToBall, double& angleToBall){
	const Pose currRobotPose = currGameState->getRobotPos( robotID );
	//const Pose ballPose = Pose( currGameState->getBallPos().getPosition() + currGameState->getBallGlobalVelocity()*this->video.getUpdateDeltaTime(),0);
	const Pose ballPose = Pose( currGameState->getBallPos().getPosition(),0);
	Vector2D ballPosition = ballPose.getPosition();

	//dystans do pilki
	//const Vector2D distToBall = Vector2D( ballPosition - currRobotPose.getPosition() );

	const double robotRotation = currRobotPose.get<2>();
	RotationMatrix rm(robotRotation);
	//pozycja pilki w ukladzie wsp zw z robotem
	Pose ballRelativePose = ballPose.transform( currRobotPose.getPosition() , rm);
	//wektor EB gdzie E to idealna pozycja pilki kiedy jest przechwycona przez robota a B pozycja pilki
	Vector2D eb (ballRelativePose.get<0>(), ballRelativePose.get<1>() - 0.08 );
    //idealna rotacja robota do celu
	//const double angle = convertAnglePI(atan2(t.get<1>(),t.get<0>()) -M_PI/2.0);
	//stara wersja
    //Vector2D oy(0.0,1.0);
    //const double angle = eb.angleTo( oy );
    //const double angle = convertAnglePI(atan2(ballRelativePose.get<1>(),ballRelativePose.get<0>()) -M_PI/2.0);
    distanceToBall = eb.length();
	// for debug infofmation
	//if( strcmp( robot.getRobotName().c_str(), "blue0" ) == 0 )
	//	file<<reference<<";"<<angle<<";"<<std::endl;

    bool robotHaveBall = false;

    Vector2D reference( cos( robotRotation+M_PI_2 ), sin( robotRotation+M_PI_2 ) );
    Vector2D toBall = ballPosition - currRobotPose.getPosition();
    double angle = toBall.angleTo(reference);


    //czy pilka jest przed dribblerem
    if ( fabs(angle) < 0.33 ){
        //czy pilka jest odpowienio blisko dribblera
        //if ( toBall.length() < ( 0.075+0.012+0.022 ) / cos(angle) ){
    	if(eb.length() < 0.015){
    	//if ( toBall.length() < ( 0.012+0.006+0.02 ) / cos(angle) ){
        	robotHaveBall = true;
        }
    }

    /*
     * to nie dziala
    //czy pilka jest przed dribblerem
	if ( fabs(angle) < 0.33 ){
		//czy pilka jest odpowienio blisko dribblera
		//promien robota + wystajacy dribbler + promien pilki
		if( distanceToBall < 0.06 + 0.04 + 0.01){
			LOG_INFO(log, "Robot has got ball. RobotPosition "<<currRobotPose<<" ball position "<< ballPosition<<" angleToBall "<<angle<<" distance to ball "<< distToBall.length());
			robotHaveBall = true;
		}
	}
	*/

	// ten kawalek kodu wyznacza kat o jaki robot musi sie obrocic zeby byc skierowanym na cel
    {	//to nie dziala
		/*RotationMatrix rm0(0);

		Pose reltargetPose_ = ballPose.transform( currRobotPose.getPosition(),rm0 );
		Pose reltargetPose = reltargetPose_*100;
		angleToBall = -atan2(reltargetPose.get<0>(),reltargetPose.get<1>()) ;
		 */
    	angleToBall = convertAnglePI(atan2(ballRelativePose.get<1>(),ballRelativePose.get<0>()) -M_PI/2.0);
    }
	return robotHaveBall;
}

//fcja wg Kamila
/*
bool EvaluationModule::isRobotOwnedBall(const Robot & robot){
//dane od KAMILA
//0.22 = kąt :) srodek robota - srodek tego grubszego elementu dribblera
//0.075 (srodek robota - srodek dribblera) +
//0.006 (promien dribblera) +
//0.022 (troche powiekszony promien pilki)

//moje dane
//0.33 = kąt :) srodek robota - srodek tego grubszego elementu dribblera
//0.075 (srodek robota - srodek dribblera) +
//0.012 (promien dribblera) +
//0.022 (troche powiekszony promien pilki)

    GameStatePtr currGameState(new GameState());

    if( video.updateGameState(currGameState) < 0 ){
		std::ostringstream s;
		s<<__FILE__<<":"<<__LINE__;
    	throw SimulationException( s.str() );
    }

    bool ballIsOwned =  false;
    Pose robotPos = currGameState->getRobotPos(robot.getRobotID() );
    Pose ballPos=currGameState->getBallPos();

    Vector2D reference( cos( robotPos.get<2>()+M_PI_2 ), sin( robotPos.get<2>()+M_PI_2 ) );
    Vector2D toBall = ballPos.getPosition() - robotPos.getPosition();
    double angle = toBall.angleTo(reference);


    //czy pilka jest przed dribblerem
    if ( fabs(angle) < 0.33 ){
        //czy pilka jest odpowienio blisko dribblera
        if ( toBall.length() < ( 0.075+0.012+0.022 ) / cos(angle) ){
            ballIsOwned = true;
        }
    }

    return ballIsOwned;
}
*/
/*
*@ zwraca najwiekszy otwarty kat prowadzacy do celu
*/
std::pair<double, double> EvaluationModule::aimAtGoal(const std::string& robotName, double& angleToShoot, double & score) const{
	//SimControl::getInstance().pause();

	GameStatePtr currGameState( new GameState() );

    if( video.updateGameState( currGameState ) < 0){
		std::ostringstream s;
		s<<__FILE__<<":"<<__LINE__;
    	throw SimulationException( s.str() );
    }

    return aimAtGoal( currGameState, robotName, angleToShoot, score);
}

std::pair<double, double> EvaluationModule::aimAtGoal(const GameStatePtr & currGameState,const std::string& robotName,double& angleToShoot,double & score_) const{

	Pose robotPose=currGameState->getRobotPos( Robot::getRobotID(robotName) );
    //LOG_INFO(log, "Wyznaczam katy do strzalu. Pozycja strzalu " << robotPose );

    //pozycje wszystkich robotow poza zadanym
    std::vector<Pose> enemyPositions=currGameState->getEnemyRobotsPos(Robot::getRobotID(robotName));

    std::vector<Pose>::iterator ii=enemyPositions.begin();

    /*Obliczam kat do bramki w zaleznosci od koloru druzyny i bramki na ktora gra
     *
     */
    //wersor osi oy
    Vector2D oy(0,1);
    double alfa1=0;
    double alfa2=0;
    double dist=0;
    double additionalRotation = 0;

    double realMinAng;
    double realMaxAng;

    if(robotName.compare(0,3,"red")==0){
    	//Pose p1(Videoserver::getBlueGoalLeftCornerPosition(),0 );
    	//Pose p2(Videoserver::getBlueGoalRightCornerPosition(),0 );

    	Vector2D v1 = Videoserver::getBlueGoalLeftCornerPosition() - robotPose.getPosition();
    	Vector2D v2 = Videoserver::getBlueGoalRightCornerPosition() - robotPose.getPosition();

    	//LOG_INFO( log," for robot "<<robotName<<" v1 "<<v1<<" v2 "<<v2 );

    	/*
    	alfa1 = p1.getPosition().angleTo(oy);
    	alfa1 = convertAnglePI(alfa1);

	    alfa2 = p2.getPosition().angleTo(oy);
	    alfa2 = convertAnglePI(alfa2);
*/
    	alfa1 = v1.angleTo(oy);
	    alfa2 = v2.angleTo(oy);

	    realMinAng = alfa1 < alfa2 ? alfa1 : alfa2;
	    realMaxAng = alfa1 > alfa2 ? alfa1 : alfa2;

    	alfa1 = convertAngle2PI(alfa1);
	    alfa2 = convertAngle2PI(alfa2);

	    double tmp2 = convertAnglePI(alfa2);
	    double tmp1 = convertAnglePI(alfa1);


	    dist = Videoserver::getBlueGoalMidPosition().distance( robotPose.getPosition() );
	    //LOG_DEBUG( log," open angle to the blue goal before translation min "<<tmp1<<" max "<<tmp2 );
	   // LOG_DEBUG( log," open angle to the blue goal before translation min "<<tmp1<<" max "<<tmp2<<" operacja odwrotna "<<" min"<<convertAnglePI(alfa1)<<" max "<<convertAnglePI(alfa2) )
        //LOG_DEBUG( log," open angle to the blue goal min "<<alfa1<<" max "<<alfa2 );

        double halfFieldLen = Config::getInstance().field.FIELD_LENGTH / 2.0;


    	if( dist > halfFieldLen  ){
    		tmp2 = alfa1 > alfa2 ? alfa1 : alfa2;
    		tmp1 = alfa1 < alfa2 ? alfa1 : alfa2;
    		//SimControl::getInstance().resume();
    		LOG_DEBUG(log, "blue goal is to far");
    		return std::pair<double,double>( tmp1, tmp2 );
    	}

  /*  	if(currGameState->getRedGoalArea()==bottom){
    	    Pose p1(appConfig.field.BOTTOM_GOAL_LEFT_CORNER.x, appConfig.field.BOTTOM_GOAL_LEFT_CORNER.y, 0);
    	    p1 = p1.translation(robotPose.getPosition());

    	    Pose p2(appConfig.field.BOTTOM_GOAL_RIGHT_CORNER.x, appConfig.field.BOTTOM_GOAL_RIGHT_CORNER.y, 0);
    	    p2 = p2.translation(robotPose.getPosition());

    	    //Vector2D v1=appConfig.field.BOTTOM_GOAL_LEFT_CORNER - robotPose.getPosition();
    	    //Vector2D v2=appConfig.field.BOTTOM_GOAL_RIGHT_CORNER - robotPose.getPosition();
    	    alfa1 = p1.getPosition().angleTo(ox);
    	    alfa2 = p2.getPosition().angleTo(ox);

    	    //alfa1 = appConfig.field.BOTTOM_GOAL_LEFT_CORNER.angleTo(ox);
    	    //alfa2 = appConfig.field.BOTTOM_GOAL_RIGHT_CORNER.angleTo(ox);

    	    dist = appConfig.field.BOTTOM_GOAL_MID_POSITION.distance( robotPose.getPosition() );

            LOG_DEBUG( log," open angle to the red  bottom goal min "<<alfa1<<" max "<<alfa2 );

    	}
    	else{
    	    Pose p0l(appConfig.field.TOP_GOAL_LEFT_CORNER.x, appConfig.field.TOP_GOAL_LEFT_CORNER.y, 0);
    	    Pose p1l = p0l.translation(robotPose.getPosition());

    	    Pose p0r(appConfig.field.TOP_GOAL_RIGHT_CORNER.x, appConfig.field.TOP_GOAL_RIGHT_CORNER.y, 0);

    	    Pose p1r = p0r.translation(robotPose.getPosition());

    	    //LOG_DEBUG( log,"robot position     " << robotPose.getPosition() );

    	    //Vector2D v1=appConfig.field.TOP_GOAL_LEFT_CORNER - robotPose.getPosition();
    	    //Vector2D v2=appConfig.field.TOP_GOAL_RIGHT_CORNER - robotPose.getPosition();

    	    alfa1 = p1l.getPosition().angleTo(ox);
    	    alfa2 = p1r.getPosition().angleTo(ox);

    	    //alfa1 = appConfig.field.TOP_GOAL_LEFT_CORNER.angleTo(ox);
        	//alfa2 = appConfig.field.TOP_GOAL_RIGHT_CORNER.angleTo(ox);
        	dist = appConfig.field.TOP_GOAL_MID_POSITION.distance( robotPose.getPosition() );

            //LOG_DEBUG( log,"!!!!!!!!!!!!!!!!!!!!!!!!!!!!! p1l"<<p0l<<" p1r "<<p0r );
            LOG_DEBUG( log," open angle to the red top goal min "<<alfa1<<" max "<<alfa2 );

    	} */
    }else{
		Vector2D v1 = Videoserver::getRedGoalLeftCornerPosition() - robotPose.getPosition();
	    Vector2D v2 = Videoserver::getRedGoalRightCornerPosition() - robotPose.getPosition();

	    additionalRotation = -M_PI;
	    //LOG_INFO( log," for robot "<<robotName<<" v1 "<<v1<<" v2 "<<v2<<" angle between "<<v1.angleTo(v2) );


    	//alfa1 = v1.angleTo(oy);
	    //alfa2 = v2.angleTo(oy);
	    Pose p1(Videoserver::getRedGoalLeftCornerPosition(),0);
	    Pose p2(Videoserver::getRedGoalRightCornerPosition(),0);
	    alfa1 = calculateProperAngleToTarget( robotPose,p1 );
	    alfa2 = calculateProperAngleToTarget( robotPose,p2 );

	    double tmp2 = alfa2;
	  	double tmp1 = alfa1;
	    dist = Videoserver::getRedGoalMidPosition().distance( robotPose.getPosition() );
        //LOG_INFO( log,"dist to goal "<<dist<<" open angle to the red goal min "<<alfa1<<" max "<<alfa2 );

	    realMinAng = alfa1 < alfa2 ? alfa1 : alfa2;
	    realMaxAng = alfa1 > alfa2 ? alfa1 : alfa2;

    	alfa1 = convertAngle2PI(alfa1);
	    alfa2 = convertAngle2PI(alfa2);



		//alfa1 = appConfig.field.BOTTOM_GOAL_LEFT_CORNER.angleTo(ox);
		//alfa2 = appConfig.field.BOTTOM_GOAL_RIGHT_CORNER.angleTo(ox);
		dist = Videoserver::getRedGoalMidPosition().distance( robotPose.getPosition() );


        double halfFieldLen = Config::getInstance().field.FIELD_LENGTH / 2.0;

		if( dist > halfFieldLen){
			//SimControl::getInstance().resume();
			tmp2 = alfa1 > alfa2 ? alfa1 : alfa2;
		    tmp1 = alfa1 < alfa2 ? alfa1 : alfa2;
		    //SimControl::getInstance().resume();
		    LOG_DEBUG(log, "red goal is to far");
		    return std::pair<double,double>( tmp1, tmp2 );
			//return std::pair<double,double>( 0, 0 );
		}

		//LOG_DEBUG( log," open angle to the red goal before translation min "<<tmp1<<" max "<<tmp2<<" operacja odwrotna "<<" min"<<convertAnglePI(alfa1)<<" max "<<convertAnglePI(alfa2) );
        //LOG_DEBUG( log,"open angle to the red bottom goal min "<<alfa1<<" max "<<alfa2 );

    	/*
		if(currGameState->getBlueGoalArea()==bottom){
			Vector2D v1=appConfig.field.BOTTOM_GOAL_LEFT_CORNER - robotPose.getPosition();
    	    Vector2D v2=appConfig.field.BOTTOM_GOAL_RIGHT_CORNER - robotPose.getPosition();
    	    alfa1 = v1.angleTo(ox);
    	    alfa2 = v2.angleTo(ox);

			//alfa1 = appConfig.field.BOTTOM_GOAL_LEFT_CORNER.angleTo(ox);
			//alfa2 = appConfig.field.BOTTOM_GOAL_RIGHT_CORNER.angleTo(ox);
			dist = appConfig.field.BOTTOM_GOAL_MID_POSITION.distance( robotPose.getPosition() );

            LOG_DEBUG( log,"open angle to the blue bottom goal min "<<alfa1<<" max "<<alfa2 );
		}
		else{
		    Vector2D v1=appConfig.field.TOP_GOAL_LEFT_CORNER - robotPose.getPosition();
    	    Vector2D v2=appConfig.field.TOP_GOAL_RIGHT_CORNER - robotPose.getPosition();
    	    alfa1 = v1.angleTo(ox);
    	    alfa2 = v2.angleTo(ox);

			//alfa1 = appConfig.field.TOP_GOAL_LEFT_CORNER.angleTo(ox);
			//alfa2 = appConfig.field.TOP_GOAL_RIGHT_CORNER.angleTo(ox);
			dist = appConfig.field.TOP_GOAL_MID_POSITION.distance( robotPose.getPosition() );

            LOG_DEBUG( log,"open angle to the blue top goal min "<<alfa1<<" max "<<alfa2 );

		}*/
    }

    double alfamin = alfa1 < alfa2 ? alfa1 : alfa2;
    double alfamax = alfa1 > alfa2 ? alfa1 : alfa2;

    Set maxOpenAngle( alfamin, alfamax, dist );

    Set tmpAng(maxOpenAngle);

    std::list< Set > angles;
    angles.push_back(maxOpenAngle);

    LOG_DEBUG(log,"maksymalny kat do celu  "<<maxOpenAngle);

    for(ii = enemyPositions.begin(); ii!=enemyPositions.end(); ii++){
    	if( ii->distance(robotPose) > Config::getInstance().getRRTRobotRadius()){
			Set ang = findObstacleCoverAngles(robotPose,*ii, additionalRotation);
			LOG_DEBUG(log, "obs pose "<<*ii<<" angles "<<ang);
			if( ang.d > 0 ){
				addToList( ang , angles);
			}
    	}
    }

    LOG_DEBUG(log,"angles.size()  "<<angles.size());

    assert( angles.size() > 0 );

    LOG_DEBUG(log,"angles.size()  "<<angles.size());

    std::list<Set>::iterator ss;
    for(ss = angles.begin(); ss!=angles.end(); ss++){
		//Set ang = findObstacleCoverAngles(robotPose,*ii);
    	LOG_DEBUG(log,"set  "<<*ss);
    }

    //znajdz najszerszy przedzial w kolekcji angles
    std::list<Set>::iterator iii=
        std::max_element(angles.begin(),angles.end(),
			  boost::bind( &Set::width,_1) );

    double min = convertAnglePI( (*iii).angmin );
    double max = convertAnglePI ( (*iii).angmax );

    alfamin = min < max ? min : max;
    alfamax = min > max ? min : max;

    /*
    double tmp;

    if( alfamin < realMinAng ){

    	alfamin = realMinAng

    	alfamin = max;
		double tmp = alfamax;
		alfamax = alfamin;
		alfamin = tmp;
    }
    */


    alfamin = realMinAng < alfamin ? alfamin : realMinAng;
    alfamax = realMaxAng > alfamax ? alfamax : realMaxAng;

    if(alfamin > alfamax){
    	double tmp = alfamax;
    	alfamax = alfamin;
    	alfamin = tmp;
    }

    //SimControl::getInstance().resume();


	double score = 0;
	double sign = 1.0;

	if( alfamin * alfamax > 0.0  ){
		score = alfamax - alfamin;
	}
	else{
		if( fabs( alfamax )  + fabs( alfamin ) > M_PI ){
			score = fabs( M_PI - fabs( alfamax )  + M_PI- fabs( alfamin ) );
			sign = -1;
		}
		else{
			score = fabs( alfamax )  + fabs( alfamin );
		}
	}

	score_=score;
	angleToShoot = convertAnglePI(alfamin + sign*score/2.0);



	//LOG_INFO(log, "exit from aimToGoal ang min "<<alfamin<<" ang max "<<alfamax<<" score "<<score<<" angleToShoot "<<angleToShoot  );

    return std::pair<double,double>( alfamin , alfamax );
}

bool EvaluationModule::addToList(Set &set, std::list<Set> &sets) const{
	BOOST_ASSERT(sets.size()>0);
	LOG_DEBUG(log,"add set To list "<<set);
	bool result = false;
	std::list<Set>::iterator i;
	double min = 0.01;
	for(i=sets.begin();i!=sets.end();i++){
		//dodawany zbior i wskazywany przez i są rozłączne
		if ((*i).areSeparated(set)){
			continue;
		}
		//zbiór i zawiera zbiór dodawany set
		if((*i).include(set)){
			if ((*i).d < set.d) break;	//dodawany jest zasłonięty przez i, nie ma sensu go uwzględniać
			//w przeciwnym razie trzeba podzielic *i
			Set copy = *i;
			if (fabs(copy.angmin - set.angmin) > min)
				sets.insert(i, Set(copy.angmin, set.angmin, copy.d));
			sets.insert(i, set);
			if (fabs(set.angmax - copy.angmax) > min)
				sets.insert(i, Set(set.angmax, copy.angmax, copy.d));
			i=sets.erase(i); i--;

			result = true;
			break;
		}
		//i zawiera się w dodawanym secie
		if((*i).isIncluded(set)){
			if ((*i).d > set.d)	//jesli set jest blizej, zmieniam wartosc d dla *i
				(*i).d = set.d;
			continue;
		}
		//czesciowe zawieranie
		if((*i).partlyInclude(set)){
			if ((*i).d < set.d) continue;	//i tak *i będzie przyslaniac set
			//w przeciwnym wypadku set jest blizej
			if ((*i).angmin < set.angmin){
				if (fabs((*i).angmin - set.angmin) > min)
					sets.insert(i,Set((*i).angmin,set.angmin,(*i).d));
				if (fabs(set.angmin - (*i).angmax) > min)
					sets.insert(i,Set(set.angmin,(*i).angmax,set.d));
				i=sets.erase(i); i--;
				result = true;
			}
			else{
				if (fabs((*i).angmin -set.angmax) > min)
					sets.insert(i,Set((*i).angmin,set.angmax,set.d));
				if (fabs(set.angmax - (*i).angmax) > min)
					sets.insert(i,Set(set.angmax,(*i).angmax,(*i).d));
				i=sets.erase(i); i--;

				result = true;
			}
			continue;
		}
	}

	return result;
}


Set EvaluationModule::findObstacleCoverAngles(Pose currRobotPose,Pose obstaclePosition, double rotation) const{

	//std::cout<<"robotPose "<<currRobotPose<<std::endl;
	//std::cout<<"obstaclePosition "<<obstaclePosition<<std::endl;

	//pozycja celu w ukladzie wsp zwiazanych z robotem
	//Pose reltargetPose=obstaclePosition.translation(currRobotPose.getPosition());
	Pose reltargetPose=obstaclePosition.transform(currRobotPose.getPosition(), rotation);
	LOG_DEBUG(log, " rotation "<<rotation );
	//std::cout<<"relTargetPose "<<reltargetPose<<std::endl;

	double x=reltargetPose.getPosition().x;
	double y=reltargetPose.getPosition().y;

	//LOG_INFO(log, "relObsPosition "<<reltargetPose.getPosition()<<" obsPosition "<<obstaclePosition<<" currRobotPose "<<currRobotPose );

	if(y<=0){
		return Set( -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity() , -std::numeric_limits<double>::infinity());

	}
	//assert(y>0);

	if( reltargetPose.getPosition().length() < Config::getInstance().getRRTRobotRadius() ){
		assert(false);
	}


	//promien okregu opisujacego przeszkode
	double obstacleRadious=Config::getInstance().getRRTRobotRadius();
	double alfa1;
	double alfa2;

	double sgn= x>0 ? 0: M_PI;

	if( fabs( fabs(x)-fabs(obstacleRadious) )< 0.001 ){

		//styczna ma rownanie x=A
		alfa1=M_PI/2.0;

		double a2=(y*y-obstacleRadious*obstacleRadious)/(2*x*y);
		//Vector2D v2(sgn, a2 );
		alfa2=atan(a2)+sgn;

	}else{
		//wspolczynniki kierunkowe prostych stycznych do okregu opisujacego przeszkode
		double a1= ( -x*y - obstacleRadious*sqrt(y*y + x*x -obstacleRadious*obstacleRadious) )/ (obstacleRadious*obstacleRadious-x*x);
		double a2= ( -x*y + obstacleRadious*sqrt(y*y + x*x -obstacleRadious*obstacleRadious) )/ (obstacleRadious*obstacleRadious-x*x);

		//LOG_INFO(log, "a1 "<<a1<<" reltargetPose"<<reltargetPose);

		//LOG_INFO(log, "a2 "<<a2<<" reltargetPose"<<reltargetPose);

		if( !boost::math::isnormal(a1) ){
			double a1= ( -x*y - obstacleRadious*sqrt( y*y + x*x -obstacleRadious*obstacleRadious ) )/ (obstacleRadious*obstacleRadious-x*x);
			LOG_DEBUG(log, "a1"<<a1<<" reltargetPose"<<reltargetPose);
			//std::cout<<<<std::endl;
		}

		if( !boost::math::isnormal(a2) ){
			double a2= ( -x*y + obstacleRadious*sqrt(y*y + x*x -obstacleRadious*obstacleRadious) )/ (obstacleRadious*obstacleRadious-x*x);
			LOG_DEBUG(log, "a2 "<<a2<<" reltargetPose"<<reltargetPose);
			//std::cout<<a2<<"reltargetPose"<<reltargetPose<<std::endl;
		}

		assert(boost::math::isnormal(a1) );
		assert(boost::math::isnormal(a2) );

		//std::cout<<"a1"<<a1<<std::endl;
		//std::cout<<"a2"<<a2<<std::endl;

		//Vector2D v1(sgn,fmin(a1,a2) );
		//Vector2D v2(sgn,fmax(a1,a2) );

		//double cosalfa=v1.angleTo(v2);

		//kat
		//alfa1=convertAnglePI( atan(a1)+sgn );
		//alfa2=convertAnglePI( atan(a2)+sgn );

		alfa1=convertAngle2PI( atan(1/a1) );
		alfa2=convertAngle2PI( atan(1/a2) );


		//double alfa3=convertAnglePI( atan(1/a1) );
		//double alfa4=convertAnglePI( atan(1/a2) );
		//LOG_DEBUG(log,"alfa3 "<<alfa3<<" alfa4 "<<alfa4);

		double check1 = fabs(a1*x-y)/sqrt(a1*a1 +1);
		double check2 = fabs(a2*x-y)/sqrt(a2*a2 +1);

		LOG_DEBUG(log,"check1 "<<check1<<" check2 "<<check2);
	}

	//katy pomiedzy ktorymi znajduje sie przeszkoda
	double alfamin = fmin(alfa1, alfa2);
	double alfamax = fmax(alfa1, alfa2);
	//std::cout<<"alfamin= "<<alfamin<<" alfamax="<<alfamax<<std::endl;
	//std::cout<<std::endl;

	//return Set(alfamin,alfamax,currRobotPose.distance(obstaclePosition));

	return Set(alfamin, alfamax,reltargetPose.getPosition().length() );

}


void EvaluationModule::test(Pose currRobotPose,Pose targetPosition){
	//findObstacleCoverAngles(currRobotPose,targetPosition);
	const std::string robotName("red0");
	//aimAtGoal(robotName);

}

EvaluationModule::~EvaluationModule()
{
    LOG_INFO(log,"destroy EvaluationModule" );
}


std::ostream & operator<<(std::ostream & os, const BallState::ballState & bState ){
	switch(bState){
	case BallState::free:
		os<<"free";
		break;
	case BallState::out:
		os<<"out";
		break;
	case BallState::in_goal:
		os<<"in_goal";
		break;
	case BallState::occupied_our:
		os<<"occupied_our";
		break;
	case BallState::occupied_theirs:
		os<<"occupied_theirs";
		break;
	case BallState::mine:
		os<<"mine";
		break;
	default:
		break;
	};

	return os;
}
