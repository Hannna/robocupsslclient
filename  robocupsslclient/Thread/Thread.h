#ifndef THREAD_H
#define THREAD_H


#include <pthread.h>

class Thread
{
    typedef pthread_t THREADID;
    public:
        Thread();
        virtual ~Thread();
        int Start(void * arg);
        THREADID getThreadID(){ return this->ThreadId_;}
        void join(){pthread_join(ThreadId_,NULL);}
    protected:
      int Run(void * arg);
      static void * EntryPoint(void*);
      /*@brief ew funkcja do przedefiniowania
      * wywolywana przed uruchomieniem watku
      *
      */
      virtual void Setup();
      /*@brief funkcja watku
      *
      */
      virtual void Execute(void*) = 0;
      inline void * Arg() const {
          return Arg_;
        }
      void Arg(void* a){Arg_ = a;}
    private:
      THREADID ThreadId_;
      pthread_attr_t attr;
      void * Arg_;
};

#endif // THREAD_H
