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
#include "../EvaluationModule/EvaluationModule.h"
//#include "../Thread/Thread.h"

#include <boost/thread/mutex.hpp>

class Task;
typedef boost::weak_ptr <Task> TaskWeakPtr;
typedef boost::shared_ptr <Task> TaskSharedPtr;
//typedef Task* TaskPtr;
//typedef boost::scoped_ptr <Task> TaskPtr;



class Task {
public:
	typedef
	enum predicate_{
		null = 0x00,
		kick_if_we_can = 0x01,
		pass = 0x02,
		should_have_ball = 0x04,
		analyse_all_field = 0x08,
		got_ball = 0x10,
		go_to_ball = 0x20,
	} predicate;

	enum status{
		not_completed = - 90,
		//sterowany robot jest w stanie kolizji
		collision,
		//kazdy inny blad
		error,
		//cel jest przesloniety, lub wew przeszkody
		target_blocked,
		ok = 0,
		kick_ok,
		get_ball
	};

public:
	Task(Robot *robot);
	virtual void stop();
	virtual ~Task();
	virtual status execute(void*, const int steps=-1);
	/*@brief zwraca wskaznik na Task wynikajacy z SSM w kolejnym kroku algorytmu
	 * lub NULL jest zmiana Task-u nie nastepuje
	 *
	 */
	virtual Task* nextTask()=0;
	/*@brief ustawia dany parametr
	 *
	 */
	void markParam(predicate p);
	/*@brief zdejmuje dany parametr
	 *
	*/
	void unmarkParam(predicate p);
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
	EvaluationModule & evaluationModule;
	GameStatePtr currGameState;

	//predytkaty ustawiane binarnie
	int predicates;
private:
	Task();
	Task(const Task &);
	Task& operator=(const Task&);

};

std::ostream & operator<<(std::ostream & os, const Task::status & status );

#endif /* TASK_H_ */
