#ifndef SHOOTTACTIC_H
#define SHOOTTACTIC_H

#include "../AbstractTactic/AbstractTactic.h"
class Robot;

class ShootTactic: public AbstractTactic
{
    public:
        ShootTactic(Robot & robot);
        virtual void execute(void *);
        virtual bool isFinish();
        virtual ~ShootTactic();
    protected:
    private:
};

#endif // SHOOTTACTIC_H
