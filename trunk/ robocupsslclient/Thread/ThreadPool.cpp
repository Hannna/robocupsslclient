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
#include <vector>
#include <sys/socket.h>
#include <boost/foreach.hpp>
#include "../Config/Config.h"
#include "../Robot/Robot.h"

Mutex ThreadPool::mutex;
ThreadPool * ThreadPool::threadPool;

ThreadPool& ThreadPool::getInstance(){
    if(ThreadPool::threadPool)
        return *ThreadPool::threadPool;
    else{
        LockGuard m(mutex);
        if(ThreadPool::threadPool)
            return *ThreadPool::threadPool;
        else{
        	int maxThreadsNr_ = Config::getInstance().getBlueTeamSize() + Config::getInstance().getRedTeamSize();
        	ThreadPool::threadPool = new ThreadPool( maxThreadsNr_ );
            return *ThreadPool::threadPool;
        }

    }
}
/** @brief create maxThreadsNr works threads with default task
 *
 */
ThreadPool::ThreadPool(const int maxThreadsNr_):maxThreadsNr(maxThreadsNr_) {

	const std::vector<std::string> blueTeam=Config::getInstance().getBlueTeam();
	BOOST_FOREACH(std::string modelName,blueTeam){
		Thread* t = new Thread();
		t->start(NULL);
		ThreadInfo ti( t, false);
		workers[Robot::getRobotID( modelName)] = ti;
	}

	const std::vector<std::string> redTeam=Config::getInstance().getRedTeam();
	BOOST_FOREACH(std::string modelName,redTeam){
		Thread* t = new Thread();
		t->start(NULL);
		ThreadInfo ti( t, false);
		workers[Robot::getRobotID( modelName)] = ti;
	}

}


ThreadPool::ThreadPool():maxThreadsNr(0){
	;
}
ThreadPool::ThreadPool(const ThreadPool& tp):maxThreadsNr(tp.maxThreadsNr){

}

bool ThreadPool::setThreadTask( Thread::ThreadTaskPtr  task, Tactic* ptr, Robot::robotID id ){
	//if( this->workers.size() < this->maxThreadsNr ){
		//task->start();
		//workers.push_back(task_id(task,joinable));
		workers[id].first->stopTask();
		std::cout<<" set thread func for robot "<<id<<std::endl;
		workers[id].first->setThreadFunc( task, ptr );
		workers[id].first->startTask();
		return true;
	//}
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
		std::map<Robot::robotID,ThreadInfo>::iterator ii=workers.begin();
		for(;ii!=workers.end();ii++){
			if( ii->second.first )
				ii->second.first->join();
			delete ii->second.first;
		}
	}

}
