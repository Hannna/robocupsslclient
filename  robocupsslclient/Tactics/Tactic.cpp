#include "Tactic.h"
#include "../EvaluationModule/EvaluationModule.h"
#include "../Thread/ThreadPool.h"


Tactic::Tactic(Robot& robot_): evaluation(EvaluationModule::getInstance() ) ,robot(robot_),
        log( getLoggerPtr (robot_.getRobotName().c_str() ) )
{
	this->stop = false;
	this->active = false;
	finished = false;
    bestScore=0;
    predicates = 0;
    pthread_cond_init (&finish_cv, NULL);
}

void Tactic::reset(){
	LockGuard l(mutex);
	this->stop = false;
	this->bestScore = 0;
	this->predicates = 0;
	this->finished = false;
	//czy jest to aktywna taktyka
	//this->active;

}

void Tactic::markParam(predicate p){
	this->predicates |=p;

}

void Tactic::unmarkParam(predicate p){

	this->predicates &= ~p;

}

void Tactic::waitForFinish(){
	LockGuard l(mutex);
	if(this->finished){
		return;
	}
	pthread_cond_wait(&this->finish_cv,mutex.get() );
}

void Tactic::start(){

	{
		LockGuard l(mutex);
		if(finished)
			return;
	}

	ThreadPool::getInstance().setThreadTask( Thread::ThreadTaskPtr(&Tactic::execute),this,this->robot.getRobotID());
	LOG_INFO(log,"Start tactic for robot "<<this->robot.getRobotName() );
	//LOG_INFO(log,"Tactic for robot "<<this->robot.getRobotName()<<" added to thread pool" );
	//std::cout<<" Tactic end "<<std::endl;
}

Tactic::~Tactic()
{
    //dtor
}
