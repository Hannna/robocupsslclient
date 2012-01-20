#include "SingleThread.h"
#include <iostream>
#include "../Exceptions/SimulationException.h"

SingleThread::SingleThread() {
	joined = false;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
}

int SingleThread::start(void * arg_)
{
   arg(arg_); // store user data
   int code = pthread_create(&threadId_,&attr,SingleThread::threadFunction, this);
   joined = false;

   return code;
}

void SingleThread::join(){
	if( !joined )
		pthread_join(threadId_,NULL);
	joined = true;
}

int SingleThread::run(void * arg_)
{
    setup();
    execute( arg_ );
    return 0;
}

void SingleThread::killThread(){
	pthread_cancel(this->threadId_);
}

void * SingleThread::threadFunction(void * pthis)
{
   SingleThread * pt = (SingleThread*)pthis;
   try{
	   pt->run( pt->arg() );
   }
   catch(SimulationException& e){
	   std::cout<<"exit from thread " << e.what()<<std::endl;
   }
   return 0;
}

void SingleThread::setup()
{
    // Do any setup here
}

void SingleThread::execute(void* arg)
{
        // Your code goes here
}

SingleThread::~SingleThread(){
    pthread_attr_destroy(&attr);
}
