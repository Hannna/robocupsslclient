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
    protected:
      int Run(void * arg);
      static void * EntryPoint(void*);
      virtual void Setup();
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
