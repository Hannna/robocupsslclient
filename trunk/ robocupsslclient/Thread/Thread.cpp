#include "Thread.h"
#include <iostream>
#include "../Exceptions/SimulationException.h"


Thread::Thread() {
	joined = false;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    pthread_cond_init (&change_tactic_cv, NULL);
	//stopTask_ = true;
	stopThread_ = false;
	arg_ = NULL;
}

int Thread::start(void * arg_)
{
   arg(arg_); // store user data
   int code = pthread_create(&threadId_,&attr,Thread::threadFunction, this);
   joined = false;

   return code;
}

void Thread::join(){
	if( !joined )
		pthread_join(threadId_,NULL);
	joined = true;
}

int Thread::run(void * arg_)
{
	bool stop = false;
	Tactic* tactic = NULL;
	while(!stop){
		setup();

		{
			LockGuard l( mutex );
			tactic = (Tactic*)this->arg_;
		}
		if(!tactic){
			//std::cout<<" tactic is null "<<std::endl;
			usleep(100);
			//continue;
		}
		else if(!tactic->isFinish()){
			LockGuard ll(changeTacticmutex);
			std::cout<<" start tactic "<<std::endl;
			( tactic->*((this)->taskPtr) )(NULL);

			{
				LockGuard l( mutex );
				this->arg_=NULL;
			}
			pthread_cond_wait(&change_tactic_cv,changeTacticmutex.get());
		}

		{
			LockGuard l( mutex );
			if(this->stopThread_){
				stop =true;
			}
		}
    //execute( arg_ );
	}
    return 0;
}

void Thread::killThread(){
	pthread_cancel(this->threadId_);
}

void * Thread::threadFunction(void * pthis)
{
	std::cout<<"start from thread "<<std::endl;
   Thread * pt = (Thread*)pthis;
   try{
	   pt->run( pt->arg() );
   }
   catch(SimulationException& e){
	   std::cout<<"exit from thread " << e.what()<<std::endl;
   }

   std::cout<<"exit from thread "<<std::endl;
   return 0;
}

void Thread::setup()
{
    // Do any setup here
}
/*
void Thread::execute(void* arg)
{
        // Your code goes here
}
*/
Thread::~Thread(){
    pthread_attr_destroy(&attr);
}
