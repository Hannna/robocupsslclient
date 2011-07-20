#include "Tactic.h"
#include "../EvaluationModule/EvaluationModule.h"

Tactic::Tactic(Robot& robot_): evaluation(EvaluationModule::getInstance() ) ,robot(robot_),
        log( getLoggerPtr (robot_.getRobotName().c_str() ) )
{
	this->stop = false;
    bestScore=0;
    predicates = 0;
}


void Tactic::markParam(predicate p){
	this->predicates |=p;

}

void Tactic::unmarkParam(predicate p){

	this->predicates &= ~p;

}

Tactic::~Tactic()
{
    //dtor
}
