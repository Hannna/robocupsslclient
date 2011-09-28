/*
 * DIsperseTactic.h
 *
 *  Created on: Aug 9, 2011
 *      Author: maciek
 */

#ifndef DISPERSETACTIC_H_
#define DISPERSETACTIC_H_


#include "Tactic.h"

class Robot;


class DisperseTactic: public Tactic {
public:
	DisperseTactic( Robot & robot );

	virtual bool isFinish();

	virtual ~DisperseTactic();

protected:
    virtual void execute(void *);
private:
    DisperseTactic();

};



#endif /* DISPERSETACTIC_H_ */
