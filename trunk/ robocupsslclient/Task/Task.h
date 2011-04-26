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
typedef boost::weak_ptr <Task> TaskWeakPtr;
typedef boost::shared_ptr <Task> TaskSharedPtr;
//typedef Task* TaskPtr;
//typedef boost::scoped_ptr <Task> TaskPtr;


class Task {
public:
	enum status{
		not_completed = - 99,
		collision,
		error,
		ok = 0
	};

public:
	Task(Robot *robot);
	virtual void stop();
	virtual ~Task();
	virtual status execute(void*, const int steps=-1);
	virtual Task* nextTask()=0;
	//virtual TaskSharedPtr & nextTask()=0;
protected:
	/*@brief metoda wykonujaca zadane polecenie
	 *
	 * @param [in] steps -> liczba krokow przez jaka wykonujemy
	 *  polecenie, jesli <0 wykonujemy az do jego zakonczenia
	 */
	virtual status run(void*, int steps=-1) = 0;
	const Videoserver & video;
	bool stopTask;
	Robot * robot;
	boost::mutex mutex_;
	const log4cxx::LoggerPtr log;
private:
	Task();
	Task(const Task &);
	Task& operator=(const Task&);
};

#endif /* TASK_H_ */
