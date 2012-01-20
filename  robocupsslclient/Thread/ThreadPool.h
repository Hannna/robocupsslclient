/*
 * ThreadPool.h
 *
 *  Created on: 2010-02-02
 *      Author: maciek
 */

#ifndef ThreadPool_H_
#define ThreadPool_H_

//#ifndef _REENTRANT
//#define _REENTRANT
#ifdef __cplusplus

#include <pthread.h>
#include <vector>
#include <limits>
#include <list>
#include <map>
#include "Thread.h"

#include "../Lock/Lock.h"
#include "../Robot/Robot.h"

/** @brief klasa reprezentujaca pule watkow
 *
 */
class ThreadPool {

typedef std::pair<Thread *,bool> ThreadInfo;
public:

    static ThreadPool & getInstance();
	/*@brief dodaje nowy watek do puli
	 *
	 * @param[in] task, nowe zadanie do wykonania
	 * @param[in] joinable, czy wątek ma byc utworzony jako dołączony
	 * jeśli tak to w destruktorze puli wątków bedziemy czekać na jego zakończenie
	 */
	bool setThreadTask( Thread::ThreadTaskPtr  task,Tactic* ptr, Robot::robotID id );

private:
	ThreadPool();
	ThreadPool(const ThreadPool& tp);
	ThreadPool(const int maxThreadsNr_);
	virtual ~ThreadPool();

	static ThreadPool * threadPool;
	//maksymalna liczba watkow do uruchomienia
	const int maxThreadsNr;
	std::map<Robot::robotID,ThreadInfo> workers;
	static Mutex mutex;
};

#endif
#endif /* ThreadPool_H_ */
