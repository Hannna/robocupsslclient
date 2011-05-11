#ifndef SHOOTTACTIC_H
#define SHOOTTACTIC_H

#include "../AbstractTactic/AbstractTactic.h"
class Robot;

class ShootTactic: public AbstractTactic
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
