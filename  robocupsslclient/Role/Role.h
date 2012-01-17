/*
 * Role.h
 *
 *  Created on: 24-05-2011
 *      Author: maciek
 */

#ifndef ROLE_H_
#define ROLE_H_

#include "../Logger/Logger.h"
#include "../Robot/Robot.h"
#include "../Lock/Lock.h"

class Tactic;

class Role {
public:
	Role( Robot* robot =NULL);

	Robot* getRobotPtr(){
		return robot;
	}

	Robot& getRobot(){
		return *robot;
	}

	void execute();
	void addTactic( Tactic * tactic);
	size_t getTacticsSize( );
	void disperse( );

	/*@brief stop executing current tactic
	 *
	 */
	void stop();

	Tactic* getCurrentTactic(){
		LockGuard lock(mutex);
		return currentTactic;
	}

	virtual ~Role();

private:
	Mutex mutex;
	Robot* robot;
	std::list<Tactic* > tactics;
	Tactic* currentTactic;
	log4cxx::LoggerPtr log;
};

#endif /* ROLE_H_ */