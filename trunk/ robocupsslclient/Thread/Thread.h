#ifndef THREAD_H
#define THREAD_H


#include <pthread.h>

class Thread
{
    typedef pthread_t THREADID;
    public:
        Thread();
        virtual ~Thread();
        int start(void * arg = NULL);
        THREADID getThreadID(){ return this->threadId_;}
        void killThread();
        void join(){pthread_join(threadId_,NULL);}
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
      void * arg_;
};

#endif // THREAD_H
