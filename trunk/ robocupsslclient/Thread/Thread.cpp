#include "Thread.h"
#include <iostream>

Thread::Thread() {
 pthread_attr_init(&attr);
}

int Thread::start(void * arg_)
{
   arg(arg_); // store user data
   int code = pthread_create(&threadId_,&attr,Thread::threadFunction, this);

   return code;
}

int Thread::run(void * arg_)
{
    setup();
    execute( arg_ );
    return 0;
}

void Thread::killThread(){
	pthread_cancel(this->threadId_);
}

void * Thread::threadFunction(void * pthis)
{
   Thread * pt = (Thread*)pthis;
   pt->run( pt->arg() );

   return 0;
}

void Thread::setup()
{
    // Do any setup here
}

void Thread::execute(void* arg)
{
        // Your code goes here
}

Thread::~Thread(){
    pthread_attr_destroy(&attr);
}
