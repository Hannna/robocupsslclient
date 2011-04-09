/*
 * Task.h
 *
 *  Created on: 2010-06-07
 *      Author: maciek
 */

#ifndef TASK_H_
#define TASK_H_
#include "../additional.h"
#include "../Robot/Robot.h"
#include "../VideoServer/Videoserver.h"
#include "../Thread/Thread.h"
#include <boost/thread/mutex.hpp>

//class Videoserver;
class Task;
typedef boost::shared_ptr <Task> TaskPtr;

class Task {

public:
	Task(Robot *robot);
	virtual void stop();
	virtual ~Task();
	virtual bool execute(void*, int steps=-1);
protected:
	/*@brief metoda wykonujaca zadane polecenie
	 *
	 * @param [in] steps -> liczba krokow przez jaka wykonujemy
	 *  polecenie, jesli <0 wykonujemy az do jego zakonczenia
	 */
	virtual bool run(void*, int steps=-1) = 0;
	const Videoserver & video;
	bool stopTask;
	Robot * robot;
	boost::mutex mutex_;
	const log4cxx::LoggerPtr log;
};

#endif /* TASK_H_ */
