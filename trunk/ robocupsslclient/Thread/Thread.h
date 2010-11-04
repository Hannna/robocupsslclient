#ifndef THREAD_H
#define THREAD_H


#include <pthread.h>

class Thread
{
    typedef pthread_t THREADID;
    public:
        Thread();
        virtual ~Thread();
        int start(void * arg);
        THREADID getThreadID(){ return this->threadId_;}
        void join(){pthread_join(threadId_,NULL);}
    protected:
      int run(void * arg);
      static void * entryPoint(void*);
      /*@brief ew funkcja do przedefiniowania
      * wywolywana przed uruchomieniem watku
      *
      */
      virtual void setup();
      /*@brief funkcja watku
      *
      */
      virtual void execute(void*) = 0;
      inline void * arg() const {
          return arg_;
        }
      void arg(void* a){arg_ = a;}
    private:
      THREADID threadId_;
      pthread_attr_t attr;
      void * arg_;
};

#endif // THREAD_H
