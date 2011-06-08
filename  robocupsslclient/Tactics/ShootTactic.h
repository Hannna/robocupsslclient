#ifndef SHOOTTACTIC_H
#define SHOOTTACTIC_H

#include "Tactic.h"
class Robot;

class ShootTactic: public Tactic
{
    public:
        ShootTactic(Robot & robot);

        virtual bool isFinish();
        virtual ~ShootTactic();
    protected:
        virtual void execute(void *);

    private:
};

#endif // SHOOTTACTIC_H
