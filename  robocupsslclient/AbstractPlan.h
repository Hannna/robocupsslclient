#ifndef ABSTRACTPLAN_H
#define ABSTRACTPLAN_H

#include "Task/Task.h"

class AbstractPlan
{
    public:
        AbstractPlan();
        void addTask();
        bool isFinished();
        void execute();
        virtual ~AbstractPlan();

    protected:
    private:
};

#endif // ABSTRACTPLAN_H
