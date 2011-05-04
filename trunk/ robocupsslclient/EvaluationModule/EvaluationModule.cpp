#include "EvaluationModule.h"
#include "../Lock/Lock.h"
#include "../additional.h"
#include "../Config/Config.h"
#include "../Set/Set.h"
#include "../Robot/Robot.h"
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
/*
*@ zwraca najwiekszy otwarty kat prowadzacy do celu
*/
std::pair<double, double> EvaluationModule::aimAtGoal(const std::string& robotName){
	GameStatePtr currGameState( new GameState() );
    video.updateGameState( currGameState );
    Pose robotPose=currGameState->getRobotPos( Robot::getRobotID(robotName) );

    std::vector<Pose> positions=currGameState->getEnemyRobotsPos(Robot::getRobotID(robotName));
    std::vector<Pose>::iterator ii=positions.begin();

    /*Obliczam kat do bramki w zaleznosci od koloru druzyny i bramki na ktora gra
     *
     */
    //wersor osi ox
    Vector2D ox(0,1);
    double alfa1;
    double alfa2;
    double dist;

    if(robotName.compare(0,3,"red")==0){
    	if(currGameState->getRedGoalArea()==bottom){
    	    Pose p1(appConfig.field.BOTTOM_GOAL_LEFT_CORNER.x, appConfig.field.BOTTOM_GOAL_LEFT_CORNER.y, 0);
    	    p1.translation(robotPose.getPosition());

    	    Pose p2(appConfig.field.BOTTOM_GOAL_RIGHT_CORNER.x, appConfig.field.BOTTOM_GOAL_RIGHT_CORNER.y, 0);
    	    p2.translation(robotPose.getPosition());

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

    	}
    }else{
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

		}
    }

    double alfamin = alfa1 < alfa2 ? alfa1 : alfa2;
    double alfamax = alfa1 > alfa2 ? alfa1 : alfa2;

    Set maxOpenAngle( alfamin, alfamax, dist );

    Set tmpAng(maxOpenAngle);

    std::list< Set > angles;
    angles.push_back(maxOpenAngle);

    for(ii = positions.begin(); ii!=positions.end(); ii++){
		Set ang = findObstacleCoverAngles(robotPose,*ii);

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

EvaluationModule::ballState EvaluationModule::getBallState(Robot::robotID id){
	GameStatePtr gameState( new GameState() );
    video.updateGameState( gameState );

    const std::vector<std::string> blueTeam=Config::getInstance().getBlueTeam();
    EvaluationModule::ballState ballState;

	//dla kazdego robota z naszej druzyny sprawdz czy nie jest posiadaczem pilki
	//dla kazdego robota z druzyny przeciwnej sprawdz czy nie jest posiadaczem pilki

    Pose p;
    Pose ballPose = gameState->getBallPos();

    if( Robot::isBlue(id) ){
    	ballState = EvaluationModule::occupied_our;
    }
    else
    	ballState = EvaluationModule::occupied_theirs;

	BOOST_FOREACH(std::string modelName,blueTeam){
		p = gameState->getRobotPos( Robot::getRobotID( modelName) );

		if( p.distance(ballPose) < ( Config::getInstance().getRobotMainCylinderRadious() + 0.04 ) ){
			return ballState;
		}
	}

    if( Robot::isBlue(id) ){
    	ballState = EvaluationModule::occupied_theirs;
    }
    else
    	ballState = EvaluationModule::occupied_our;

	const std::vector<std::string> redTeam=Config::getInstance().getRedTeam();
	BOOST_FOREACH(std::string modelName,redTeam){
		p = gameState->getRobotPos( Robot::getRobotID( modelName ) );

		if( p.distance(ballPose) < ( Config::getInstance().getRobotMainCylinderRadious() + 0.04 ) ){
			return ballState;
		}

	}

	//sprawdz czy pilka jest w obrebie boiska
	if( ( ballPose.getPosition().x >= ( Config::getInstance().field.FIELD_BOTTOM_LEFT_CORNER.x + Config::getInstance().field.FIELD_MARIGIN ) ) &&
			( ballPose.getPosition().x  <= ( Config::getInstance().field.FIELD_TOP_RIGHT_CORNER.x - Config::getInstance().field.FIELD_MARIGIN)  ) ){
		if( ( ballPose.getPosition().y <= ( Config::getInstance().field.FIELD_TOP_RIGHT_CORNER.y - Config::getInstance().field.FIELD_MARIGIN  ) ) &&
				( ballPose.getPosition().y  >= ( Config::getInstance().field.FIELD_BOTTOM_LEFT_CORNER.y + Config::getInstance().field.FIELD_MARIGIN ) ) ){
			return EvaluationModule::free;
		}
	}

	//sprawdz czy nie jest w  dolnej bramce
	if( ballPose.getPosition().y  <= ( Config::getInstance().field.FIELD_BOTTOM_LEFT_CORNER.y +Config::getInstance().field.FIELD_MARIGIN ) ) {
		if( ( ballPose.getPosition().x >= Config::getInstance().field.BOTTOM_GOAL_LEFT_CORNER.x  ) &&
				( ballPose.getPosition().x  <= Config::getInstance().field.BOTTOM_GOAL_RIGHT_CORNER.x  ) ){

				return EvaluationModule::in_goal;
		}
	}

	//sprawdz czy nie jest w  gornej bramce
	if( ballPose.getPosition().y  >= ( Config::getInstance().field.FIELD_TOP_RIGHT_CORNER.y - Config::getInstance().field.FIELD_MARIGIN ) ) {
		if( ( ballPose.getPosition().x >= Config::getInstance().field.TOP_GOAL_LEFT_CORNER.x  ) &&
				( ballPose.getPosition().x  <= Config::getInstance().field.TOP_GOAL_RIGHT_CORNER.x  ) ){

				return EvaluationModule::in_goal;
		}
	}

	//pilka jest na aucie
	return EvaluationModule::out;
}

void EvaluationModule::addToList(Set &set, std::list<Set> &sets){
	BOOST_ASSERT(sets.size()>0);
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
			}
			else{
				if (fabs((*i).angmin -set.angmax) > min)
					sets.insert(i,Set((*i).angmin,set.angmax,set.d));
				if (fabs(set.angmax - (*i).angmax) > min)
					sets.insert(i,Set(set.angmax,(*i).angmax,(*i).d));
				i=sets.erase(i); i--;
			}
			continue;
		}
	}
}

/*@brief znajduje najbardziej atrakcyjny punkt na planszy
*
* implementacja naiwna, najbardziej atrakcyjny jest srodek planszy
*/
Pose EvaluationModule::findBestDribbleTarget(){

    return appConfig.field.FIELD_MIDDLE_POSE;

}

/*
*@brief sprawdza czy robot jest w posiadaniu pilku
*
*@ret true jesli robot jest w posiadaniu pilki
*/
bool EvaluationModule::haveBall_1(const Robot & robot){
    GameStatePtr currGameState(new GameState());
    video.updateGameState(currGameState);

    Pose robotPose=currGameState->getRobotPos( robot.getRobotID() );
    Pose ballPose=currGameState->getBallPos();


    RotationMatrix rmY( robotPose.get<2>() );
    //pozycja celu w ukladzie wsp zwiazanych z robotem
    Pose ballRelPose=ballPose.transform(robotPose.getPosition(),rmY);

    //rotacja do celu
    double toBallRot=atan2( (ballRelPose.get<1>()) , (ballRelPose.get<0>()));

    return fabs(toBallRot) < ROTATION_PRECISION ? true : false ;
}

bool EvaluationModule::isRobotOwnedBall(const Robot * robot){
	return isRobotOwnedBall( *robot );
}
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
    video.updateGameState(currGameState);
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

Set EvaluationModule::findObstacleCoverAngles(Pose currRobotPose,Pose obstaclePosition){

	//std::cout<<"robotPose "<<currRobotPose<<std::endl;
	//std::cout<<"obstaclePosition "<<obstaclePosition<<std::endl;

	//pozycja celu w ukladzie wsp zwiazanych z robotem
	Pose reltargetPose=obstaclePosition.translation(currRobotPose.getPosition());

	//std::cout<<"relTargetPose "<<reltargetPose<<std::endl;

	double x=reltargetPose.getPosition().x;
	double y=reltargetPose.getPosition().y;

	if(y<=0){
		return Set( -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity() , -std::numeric_limits<double>::infinity());

	}
	//assert(y>0);

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

		assert(boost::math::isnormal(a1) );
		assert(boost::math::isnormal(a2) );

		//std::cout<<"a1"<<a1<<std::endl;
		//std::cout<<"a2"<<a2<<std::endl;

		//Vector2D v1(sgn,fmin(a1,a2) );
		//Vector2D v2(sgn,fmax(a1,a2) );

		//double cosalfa=v1.angleTo(v2);

		//kat
		alfa1=atan(a1)+sgn;
		alfa2=atan(a2)+sgn;
	}

	//katy pomiedzy ktorymi znajduje sie przeszkoda
	double alfamin = fmin(alfa1, alfa2);
	double alfamax = fmax(alfa1, alfa2);
	//std::cout<<"alfamin= "<<alfamin<<" alfamax="<<alfamax<<std::endl;
	//std::cout<<std::endl;

	return Set(alfamin,alfamax,currRobotPose.distance(obstaclePosition));
}

void EvaluationModule::test(Pose currRobotPose,Pose targetPosition){
	//findObstacleCoverAngles(currRobotPose,targetPosition);
	const std::string robotName("red0");
	aimAtGoal(robotName);

}

EvaluationModule::~EvaluationModule()
{
    //dtor
}


std::ostream & operator<<(std::ostream & os, const EvaluationModule::ballState & bState ){
	switch(bState){
	case EvaluationModule::free:
		os<<"free";
		break;
	case EvaluationModule::out:
		os<<"out";
		break;
	case EvaluationModule::in_goal:
		os<<"in_goal";
		break;
	case EvaluationModule::occupied_our:
		os<<"occupied_our";
		break;
	case EvaluationModule::occupied_theirs:
		os<<"occupied_theirs";
		break;
	default:
		break;
	};
}
