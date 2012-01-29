#ifndef THREAD_H
#define THREAD_H


#include <pthread.h>
#include "../Lock/Lock.h"
#include "../Tactics/Tactic.h"

class Thread
{
    typedef pthread_t THREADID;


    public:
    	typedef void * (Tactic::*ThreadTaskPtr)(void *) ;
        Thread();
        virtual ~Thread();
        int start(void * arg = NULL);
        THREADID getThreadID(){ return this->threadId_;}

        void stopThread(){
         	LockGuard l( mutex );
         	stopThread_ = true;
         }

        //void stopTask(){
        //	LockGuard l(mutex);
        //	stopTask_ = true;
        //}
        void setThreadFunc( ThreadTaskPtr taskPtr_, Tactic * ptr){
        	LockGuard ll(changeTacticmutex);
        	std::cout<<" set thread func 1"<<std::endl;
        	LockGuard l(mutex);
        	taskPtr = taskPtr_;
        	arg( (void *) ptr );
        	std::cout<<" set thread func 2"<<std::endl;
        	pthread_cond_signal(&change_tactic_cv);
        }
        /*void startTask(){
        	LockGuard l(mutex);
            stopTask_ = false;
        }*/
        void killThread();
        void join();

    protected:
      /*@brief ew funkcja do przedefiniowania
      * wywolywana przed uruchomieniem watku
      *
      */
      virtual void setup();
      /*@brief funkcja watku
      *
      */
      //virtual void execute(void*) = 0;
    private:
      inline void * arg() const {
            return arg_;
          }
      void arg(void* a){arg_ = a;}

      int run(void * arg);
      static void * threadFunction(void*);

      ThreadTaskPtr taskPtr;
      THREADID threadId_;
      Mutex mutex;
      Mutex changeTacticmutex;
      pthread_cond_t change_tactic_cv;
      bool stopThread_;
     // bool stopTask_;
      pthread_attr_t attr;
      bool joined;
      void * arg_;
};

#endif // THREAD_H
