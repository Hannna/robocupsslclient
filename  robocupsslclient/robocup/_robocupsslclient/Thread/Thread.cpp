#include "Thread.h"
#include <iostream>

Thread::Thread() {
 pthread_attr_init(&attr);
}

int Thread::Start(void * arg)
{
   Arg(arg); // store user data
   int code = pthread_create(&ThreadId_,&attr,Thread::EntryPoint, this);

   return code;
}

int Thread::Run(void * arg)
{
    std::cout<<"Thread::Run"<<std::endl;
    Setup();
    Execute( arg );
    return 0;
}

void * Thread::EntryPoint(void * pthis)
{
   Thread * pt = (Thread*)pthis;
   pt->Run( pt->Arg() );

   return 0;
}

void Thread::Setup()
{
    // Do any setup here
}

void Thread::Execute(void* arg)
{
        // Your code goes here
}

Thread::~Thread(){
    pthread_attr_destroy(&attr);
}
