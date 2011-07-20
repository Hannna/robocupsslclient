/*
 * Rotate.h
 *
 *  Created on: 07-05-2011
 *      Author: maciek
 */

#ifndef ROTATE_H_
#define ROTATE_H_

#include "Task.h"

class Rotate : public Task{
public:
	Rotate( const Vector2D targetPose,Robot * robot  );
	virtual Task* nextTask();
	virtual ~Rotate();
protected:
	virtual Task::status run(void * arg, int steps=-1);
private:
	Rotate();
	Rotate(const Rotate & r);
	Rotate& operator=(const Rotate& r);
	//pozycja do ktorej robot ma byc oborcony przodem
	const Vector2D targetPosition;
};

#endif /* ROTATE_H_ */
