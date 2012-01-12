/*
 * ThreadPool.cpp
 *
 *  Created on: 2010-02-02
 *      Author: maciek
 */

#include "ThreadPool.h"

#include <iostream>
#include <sstream>
#include <list>
#include <sys/socket.h>

/** @brief create maxThreadsNr works threads with default task
 *
 */
ThreadPool::ThreadPool(const int maxThreadsNr_):maxThreadsNr(maxThreadsNr_) {

}


ThreadPool::ThreadPool():maxThreadsNr(0){
	;
}
ThreadPool::ThreadPool(const ThreadPool& tp):maxThreadsNr(tp.maxThreadsNr){

}

bool ThreadPool::addTask(Thread*  task,bool joinable){
	if( this->workers.size() < this->maxThreadsNr ){
		task->start();
		workers.push_back(task_id(task,joinable));
		return true;
	}
	return false;
}

ThreadPool::~ThreadPool() {
	/*
	std::list<thread_id>::iterator ii=tids.begin();
	for(;ii!=tids.end();ii++){
	    pthread_t tid = (*ii).first;
		if((*ii).second) {
		    //LOG_INFO(getLogger(SM_ID), "waiting for thread: " << std::hex << tid);
            int r = pthread_join(tid, NULL);
            //if (r) {
             //   LOG_ERROR(getLogger(SM_ID), "on join thread: " << r);
            //}
		}
		else {
		    //LOG_INFO(getLogger(SM_ID), "NOT waiting for thread: " << std::hex << tid);
        }
	}
	*/

	{
		std::list<task_id>::iterator ii=workers.begin();
		for(;ii!=workers.end();ii++){
			if((*ii).second)
				(*ii).first->join();
			delete (*ii).first;
		}
	}

}
