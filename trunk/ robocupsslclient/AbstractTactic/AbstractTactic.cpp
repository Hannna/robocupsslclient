#include "AbstractTactic.h"
#include "../EvaluationModule/EvaluationModule.h"

AbstractTactic::AbstractTactic(Robot& robot_): evaluation(EvaluationModule::getInstance() ) ,robot(robot_),
        log(getLoggerPtr (robot_.getRobotName().c_str() ) )
{
    bestScore=0;
}

AbstractTactic::~AbstractTactic()
{
    //dtor
}
