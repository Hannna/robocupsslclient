/*
 * Experiments.h
 *
 *  Created on: Sep 27, 2011
 *      Author: maciek
 */

#ifndef EXPERIMENTS_H_
#define EXPERIMENTS_H_

#include "../Thread/Thread.h"

namespace Experiments{
 typedef   enum ExperimentKind_{
            none=-1,
            //nawigacja w dynamicznym srodowisku 2011
            navigation_2011 =0
            } ExperimentKind;
}

class Experiment :public Thread{

public:
	Experiment();
	/*@brief wykonuje czynnosci przygotowawcze do eksperymentu
	 *
	 * np. rozstawienie robotów na planszy itp
	 *
	 */
	void prepare();

	void startExperiment();

    virtual ~Experiment();
protected:
    bool stop;
    virtual void execute(void*);

};
#endif /* EXPERIMENTS_H_ */
