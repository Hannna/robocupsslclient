#include "Tactic.h"
#include "../EvaluationModule/EvaluationModule.h"
#include "../Thread/ThreadPool.h"


Tactic::Tactic(Robot& robot_): evaluation(EvaluationModule::getInstance() ) ,robot(robot_),
        log( getLoggerPtr (robot_.getRobotName().c_str() ) )
{
	this->stop = false;
	finished = false;
    bestScore=0;
    predicates = 0;
    pthread_cond_init (&finish_cv, NULL);
}


void Tactic::markParam(predicate p){
	this->predicates |=p;

}

void Tactic::unmarkParam(predicate p){

	this->predicates &= ~p;

}

void Tactic::waitForFinish(){
	LockGuard l(mutex);
	pthread_cond_wait(&this->finish_cv,mutex.get() );
}

void Tactic::start(){
	//std::cout<<" Tactic start "<<std::endl;
	{
		LockGuard l(mutex);
		if(finished)
			return;
	}

	ThreadPool::getInstance().setThreadTask( Thread::ThreadTaskPtr(&Tactic::execute),this,this->robot.getRobotID());
	//std::cout<<" Tactic end "<<std::endl;
}

Tactic::~Tactic()
{
    //dtor
}
