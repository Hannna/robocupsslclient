#include "EvaluationModule.h"
#include "../Lock/Lock.h"
#include "../additional.h"
#include <math.h>

EvaluationModule * EvaluationModule::ptr=NULL;
Mutex EvaluationModule::mutex;
//using namespace boost::math;

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

EvaluationModule::EvaluationModule():video(Videoserver::getInstance())
{

}
/*
*@ zwraca najwiekszy otwarty kat prowadzacy do celu
*/

std::pair<double, double> EvaluationModule::aimAtGoal(const std::string& robotName){
	GameStatePtr currGameState(new GameState());
    video.updateGameState(currGameState);
    Pose robotPose=currGameState->getRobotPos(robotName);

    std::vector< std::pair<double, double> > angles;
    std::vector<Pose> positions=GameState::getEnemyRobotsPos(robotName);

    //TODO: zainicjowac katem do bramki
    std::pair<double, double> maxOpenAngle;

    std::pair<double, double> tmpAng;
    for(ii = positions.begin(); ii!=positions.end(); ii++){
    	//jesli odleglosc do bramki jest mniejsza
    	//niz sterowano robota
    	//ew mozna dodac to sprawdzanie w findObstacleCoverAngles i chyba tak bedzie rozsadniej
    	if( (*ii).){
			std::pair<double, double> ang = findObstacleCoverAngles(robotPose,*ii);

			//jesli ang zawiera sie w maxOpenAngle
			if(ang.first  >   maxOpenAngle.first ){


			}
    	}
    }



    return maxOpenAngle;
}
/*
std::pair<double, double> EvaluationModule::aimAtGoal(const std::string &robotName){
    //Pose pose=gameState->getRobotPos(robotName);
    GameStatePtr currGameState(new GameState());
    video.updateGameState(currGameState);

    Pose robotPose=currGameState->getRobotPos(robotName);


    Vector2D v1 = BOTTOM_GOAL_MID_POSITION - GOAL_CORNER_LEFT_SHIFT - robotPose.getPosition();
    Vector2D v2 = BOTTOM_GOAL_MID_POSITION - GOAL_CORNER_RIGHT_SHIFT - robotPose.getPosition();

    //BOTTOM_GOAL_POSITION
    //TOP_GOAL_POSITION

    Vector2D vox(1,0);

    double alfa1=vox.angleTo(v1);
    double alfa2=vox.angleTo(v2);

    std::cout<<"alfa1 "<<alfa1<<"alfa2 "<<alfa2<<std::endl;


    //evaluation::score score=v1.scalarProduct(v2);
    evaluation::score score = fabs(alfa1-alfa2);
    return std::pair<double, double>(fmin(alfa1,alfa2), fmax(alfa1,alfa2 ) );
}
*/

/*@brief znajduje najbardziej atrakcyjny punkt na planszy
*
* implementacja naiwna, najbardziej atrakcyjny jest srodek planszy
*/
Pose EvaluationModule::findBestDribbleTarget(){

    return FIELD_MIDDLE_POSE;

}

/*
*@brief sprawdza czy robot jest w posiadaniu pilku
*
*@ret true jesli robot jest w posiadaniu pilki
*/
bool EvaluationModule::haveBall_1(const Robot & robot){
    GameStatePtr currGameState(new GameState());
    video.updateGameState(currGameState);

    Pose robotPose=currGameState->getRobotPos(robot.getRobotName());
    Pose ballPose=currGameState->getBallPos();


    RotationMatrix rmY( robotPose.get<2>() );
    //pozycja celu w ukladzie wsp zwiazanych z robotem
    Pose ballRelPose=ballPose.transform(robotPose.getPosition(),rmY);

    //rotacja do celu
    double toBallRot=atan2( (ballRelPose.get<1>()) , (ballRelPose.get<0>()));

    return fabs(toBallRot) < ROTATION_PRECISION ? true : false ;
}

bool EvaluationModule::haveBall_2(const Robot & robot){
//0.22 = kąt :) srodek robota - srodek tego grubszego elementu dribblera
//0.075 (srodek robota - srodek dribblera) + 0.006 (promien dribblera) +
//0.022 (troche powiekszony promien pilki)
    GameStatePtr currGameState(new GameState());
    video.updateGameState(currGameState);
    bool ballIsOwned =  false;
    Pose p = currGameState->getRobotPos(robot.getRobotName() );
    Pose ballPos=currGameState->getBallPos();

    Vector2D reference(cos(p.get<2>()+M_PI_2), sin(p.get<2>()+M_PI_2));
    Vector2D toBall = ballPos.getPosition() - p.getPosition();
    double angle = toBall.angleTo(reference);
//<-0.02->
//(ball)                                                          | 0.020 ball radius
//-----           -----                                   |- 0.006 thicker dribbler bar radius
//|       |-------|       |   <- dribbler         |-
//-----           -----                                   |0.075  distance from robot to dribbler center
//x (robot center)                        |
//LOG4CXX_DEBUG(logger, "Model "<<IdMgr::instance().getName(i->first));
//LOG4CXX_DEBUG(logger, "Angle: "<<angle);

    if (fabs(angle) < 0.22){        //from robot dimensions - this means ball
        //is near the dribbler, in front
        if (toBall.length() < (0.075+0.006+0.022) / cos(angle)){        //some
        //additional margin added
            ballIsOwned = true;
            //info.isOwned = true;
            //info.owner = i->first;
            //info.team = AppConfig::instance().getTeamName(i->first);
            //break;
        }
    }

    return ballIsOwned;
}

std::pair<double, double> EvaluationModule::findObstacleCoverAngles(Pose currRobotPose,Pose obstaclePosition){

	std::cout<<"currRobotPose "<<currRobotPose<<std::endl;
	std::cout<<"targetPosition "<<targetPosition<<std::endl;

	//pozycja celu w ukladzie wsp zwiazanych z robotem
	Pose reltargetPose=targetPosition.translation(currRobotPose.getPosition());

	double x=reltargetPose.getPosition().x;
	double y=reltargetPose.getPosition().y;

	assert(y>0);

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
	std::cout<<"alfamin= "<<alfamin<<" alfamax="<<alfamax<<std::endl;
	std::cout<<std::endl;


}
void EvaluationModule::test(Pose currRobotPose,Pose targetPosition){
	findObstacleCoverAngles(currRobotPose,targetPosition);

}

EvaluationModule::~EvaluationModule()
{
    //dtor
}
