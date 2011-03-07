#include "AbstractTactic.h"
#include "../EvaluationModule/EvaluationModule.h"

AbstractTactic::AbstractTactic(Robot& robot_): evaluation(EvaluationModule::getInstance() ) ,robot(robot_)
{
    bestScore=0;
}

AbstractTactic::~AbstractTactic()
{
    //dtor
}
