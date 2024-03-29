#ifndef SINGLE_THREAD_H
#define SINGLE_THREAD_H


#include <pthread.h>

class SingleThread
{
    typedef pthread_t THREADID;
    public:
    SingleThread();
	virtual ~SingleThread();
	virtual int start(void * arg = NULL);
	THREADID getThreadID(){ return this->threadId_;}
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
      virtual void execute(void*) = 0;
    private:
      inline void * arg() const {
            return arg_;
          }
      void arg(void* a){arg_ = a;}

      int run(void * arg);
      static void * threadFunction(void*);

      THREADID threadId_;
      pthread_attr_t attr;
      bool joined;
      void * arg_;
};

#endif // THREAD_H
