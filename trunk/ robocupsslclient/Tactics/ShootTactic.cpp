#include "ShootTactic.h"
#include "../EvaluationModule/EvaluationModule.h"
ShootTactic::ShootTactic(const Robot & robot): AbstractTactic(robot)
{

}


void ShootTactic::execute(){
//    GameStatePtr currGameState( new GameState() );
//    video.updateGameState(currGameState);

    //EvaluationModule& evaluation=EvaluationModule::getInstance();
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


}

bool  ShootTactic::isFinish(){

    return true;
}

ShootTactic::~ShootTactic()
{

}
