#include "EvaluationModule.h"
#include "../Lock/Lock.h"
#include "../additional.h"
#include <math.h>

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

EvaluationModule::EvaluationModule():video(Videoserver::getInstance())
{

}
/*
*@ zwraca najwiekszy otwarty kat prowadzacy do celu
*/
/*
std::pair<double, double> EvaluationModule::aimAtGoal(const std::string& robotName){
    GameStatePtr currGameState(new GameState());
    video.updateGameState(currGameState);
    Pose pose=currGameState->getRobotPos(robotName);

    Vector2D v1 = BOTTOM_GOAL_MID_POSITION - GOAL_CORNER_LEFT_SHIFT - pose.getPosition();
    Vector2D v2 = BOTTOM_GOAL_MID_POSITION - GOAL_CORNER_RIGHT_SHIFT - pose.getPosition();

    //BOTTOM_GOAL_POSITION
    //TOP_GOAL_POSITION

    evaluation::score score=v1.scalarProduct(v2);

    return score;
}
*/
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
void EvaluationModule::test(){

Pose robotPose=FIELD_MIDDLE_POSE;

//Vector2D v(1,0);
//Vector2D u(0,1);

//std::cout<<"v*u "<<v.scalarProduct(u)<<std::endl;


RotationMatrix rmY(0);
Pose currRobotPose=Pose(1,1,0);
Pose targetPosition=Pose(3,3,0);

aimAtGoal("red0" );
//pozycja celu w ukladzie wsp zwiazanych z robotem
Pose reltargetPose=targetPosition.transform(currRobotPose.getPosition(),rmY);

double x=reltargetPose.getPosition().x;
double y=reltargetPose.getPosition().y;

double r=1;

double a1= ( -x*y - r*sqrt(y*y + x*x -r*r) )/ (r*r-x*x);
double a2= ( -x*y + r*sqrt(y*y + x*x -r*r) )/ (r*r-x*x);

Vector2D v1(1,fmin(a1,a2) );
Vector2D v2(1,fmax(a1,a2) );

double cosalfa=v1.angleTo(v2);

double alfa1=atan(fmin(a1,a2));
double alfa2=atan(fmax(a1,a2));

std::cout<<"alfa1= "<<alfa1<<" alfa2="<<alfa2<<std::endl;

std::cout<<"v1= "<<v1<<" v2="<<v2<<" cosalfa"<<cosalfa<<std::endl;

/*
std::cout<<"score from middle "<<EvaluationModule::aimAtGoal(robotPose)<<std::endl;
std::cout<<"score from middle -(1,1,0) "<<EvaluationModule::aimAtGoal(robotPose-Pose(1.0,1.0,0))<<std::endl;
std::cout<<"score from middle -(2,2,0) "<<EvaluationModule::aimAtGoal(robotPose-Pose(2.0,2.0,0))<<std::endl;

std::cout<<"score from middle +(1,1,0) "<<EvaluationModule::aimAtGoal(robotPose-Pose(1.0,1.0,0))<<std::endl;
std::cout<<"score from middle +(2,2,0) "<<EvaluationModule::aimAtGoal(robotPose-Pose(2.0,2.0,0))<<std::endl;

std::cout<<"score from middle -(2,0,0) "<<EvaluationModule::aimAtGoal(robotPose-Pose(2.0,0.0,0))<<std::endl;
std::cout<<"score from middle -(3,0,0) "<<EvaluationModule::aimAtGoal(robotPose-Pose(3.0,0.0,0))<<std::endl;

std::cout<<"score from middle +(2,0,0) "<<EvaluationModule::aimAtGoal(robotPose+Pose(2.0,0.0,0))<<std::endl;
std::cout<<"score from middle +(3,0,0) "<<EvaluationModule::aimAtGoal(robotPose+Pose(3.0,0.0,0))<<std::endl;

std::cout<<"score from middle -(0,2.0,0) "<<EvaluationModule::aimAtGoal(robotPose-Pose(0.0,2.0,0))<<std::endl;
std::cout<<"score from middle -(0,3.0,0) "<<EvaluationModule::aimAtGoal(robotPose-Pose(0.0,3.0,0))<<std::endl;

std::cout<<"score from middle +(0,2.0,0) "<<EvaluationModule::aimAtGoal(robotPose+Pose(0.0,2.0,0))<<std::endl;
std::cout<<"score from middle +(0,3.0,0) "<<EvaluationModule::aimAtGoal(robotPose+Pose(0.0,3.0,0))<<std::endl;
*/
}

EvaluationModule::~EvaluationModule()
{
    //dtor
}
