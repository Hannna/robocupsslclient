#include "ShootTactic.h"
#include "../EvaluationModule/EvaluationModule.h"
#include "../Task/Task.h"
#include "../Task/GoToPose.h"
#include "../Task/KickBall.h"



ShootTactic::ShootTactic(const Robot & robot): AbstractTactic(robot)
{

}


void ShootTactic::execute(){
//    GameStatePtr currGameState( new GameState() );
//    video.updateGameState(currGameState);

    EvaluationModule& evaluation=EvaluationModule::getInstance();
    std::pair<double, double> ang=evaluation.aimAtGoal(robot.getRobotName());

    std::cout<<"angmin "<<ang.first<<" angmax"<<ang.second<<std::endl;

    //jesli kat do strzalu jest mniejszy niz 30 stopni jedz do bramki
    while( fabs( ang.first - ang.second ) < EvaluationModule::minOpenAngle ){
        Task* task= new goToPose( Config::getInstance().field.BOTTOM_GOAL_MID_POSITION
                                 ,robot);
        goToPose.execute();
    }

    /*
    //sprawdz czy dany robot ma pilke
    if(evaluation.haveBall_1(robot)){
        std::pair<double, double> ang=evaluation.aimAtGoal(robot.getRobotName());
        if( fabs(ang.first - ang.second ) > M_PI/6.0 ){

            robot.kick();

        }


    }
    else{
        //gotpballTask
    ;
    }
*/

}

bool  ShootTactic::isFinish(){

    return true;
}

ShootTactic::~ShootTactic()
{

}
