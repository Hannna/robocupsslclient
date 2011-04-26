#ifndef QUEUE_H_
#define QUEUE_H_

#include <deque>
#include<pthread.h>
#include <iostream>
#include <time.h>

#include <exception>

class SimpleBlockingQueueTakeException : public std::exception
{
public:
	SimpleBlockingQueueTakeException(){
		;
	};

  virtual const char* what() const throw()
  {
    return "Queue is empty.Can't take element";
  }

  virtual ~SimpleBlockingQueueTakeException() throw(){
	  ;
  };

};

template<typename T> class SimpleBlockingQueue {
    public:
        SimpleBlockingQueue() {
            pthread_mutex_init(&mutex, NULL);
            pthread_cond_init (&cv, NULL);
        }
        ~SimpleBlockingQueue() {

            pthread_mutex_destroy(&mutex);
            pthread_cond_destroy(&cv);
        }

        size_t offer(T element) {
            pthread_mutex_lock(&mutex);
            queue.push_back(element);
            size_t size=queue.size();
            pthread_cond_signal(&cv);
            pthread_mutex_unlock(&mutex);
            return size;
        }

        T take() {
            pthread_mutex_lock(&mutex);
            while (queue.size() == 0) {
                pthread_cond_wait(&cv, &mutex);
            }

            T returnVal = queue.front();
            queue.pop_front();
            pthread_mutex_unlock(&mutex);
            return returnVal;
        }

        /**
         * Zwraca info czy wyjeto cos z kolejki,
         * obiekt wyjety zwracany jest przez wskaznik
         */
        bool timed_take(int sec, T *elem) {
            bool r = false;
            pthread_mutex_lock(&mutex);
            struct timespec abstime;
            abstime.tv_sec=sec;
            abstime.tv_nsec=0;

            if (queue.size() == 0) {
            	 pthread_cond_timedwait(&cv,&mutex, &abstime);
            }
            if(queue.size() != 0){
                T returnVal = queue.front();
                queue.pop_front();
                *elem = returnVal;
                r = true;
            }
            pthread_mutex_unlock(&mutex);
            return r;
        }



        size_t size(){
        	pthread_mutex_lock(&mutex);
        	return this->queue.size();
        	pthread_mutex_unlock(&mutex);
        }
    private:
        std::deque<T> queue;
        pthread_mutex_t mutex;
        pthread_cond_t cv;
};

#endif
