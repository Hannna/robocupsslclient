#ifndef _PTHREAD_HPP
#define _PTHREAD_HPP


#include <pthread.h>
#include <signal.h>
#include <memory>

#define __PTHREAD_NO_EXCEPTION__

#ifndef __PTHREAD_NO_EXCEPTION__
#include <exception>
#define BLOW_SIGNAL SIGUSR1
#endif

class Mutex
{
  private:
    pthread_mutex_t mutex;

  friend class LockGuard;

  public:
  	pthread_mutex_t* get(){
  		return &mutex;
  	}
    inline Mutex();

    inline ~Mutex();

};

inline Mutex::Mutex():
    mutex((pthread_mutex_t)PTHREAD_MUTEX_INITIALIZER)
{
    //pthread_mutex_init( &mutex, NULL );
}

inline Mutex::~Mutex()
{
    pthread_mutex_destroy( &mutex );
}

/*###################################################################################################
#
####################################################################################################*/
class LockGuard {
  private:
    inline LockGuard();
  public:
    inline LockGuard( Mutex & mutex );

    inline LockGuard( pthread_mutex_t & mutex );

    inline ~LockGuard();

    inline LockGuard & operator=( Mutex & mutex );

  private:
        inline LockGuard(const LockGuard&);
        inline LockGuard & operator=( const LockGuard & );
        pthread_mutex_t* pmutex;

};

inline LockGuard::LockGuard(): pmutex(0)
{

}

inline LockGuard::LockGuard(Mutex & mutex): pmutex(&mutex.mutex)
{
    pthread_mutex_lock( pmutex );
}

inline LockGuard::LockGuard( pthread_mutex_t & mutex ):pmutex(&mutex){
    pthread_mutex_lock( pmutex );
}

inline LockGuard::~LockGuard()
{
    if(pmutex) pthread_mutex_unlock( pmutex );
}

inline LockGuard & LockGuard::operator=( Mutex & mutex )
{
    if(pmutex) pthread_mutex_unlock(pmutex);
    pthread_mutex_lock( pmutex = &mutex.mutex );
    return *this;
}

#endif
