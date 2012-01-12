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
#include "Thread.h"

/** @brief klasa reprezentujaca pule watkow
 *
 */
class ThreadPool {

typedef std::pair<Thread *,bool> task_id;
public:
	ThreadPool(const int maxThreadsNr_);
	virtual ~ThreadPool();
	/*@brief dodaje nowy watek do puli
	 *
	 * @param[in] task, nowe zadanie do wykonania
	 * @param[in] joinable, czy wątek ma byc utworzony jako dołączony
	 * jeśli tak to w destruktorze puli wątków bedziemy czekać na jego zakończenie
	 */
	bool addTask(Thread* task,bool joinalbe);

private:
	ThreadPool();
	ThreadPool(const ThreadPool& tp);
	//maksymalna liczba watkow do uruchomienia
	const int maxThreadsNr;
	std::list<task_id> workers;
};

#endif
#endif /* ThreadPool_H_ */
