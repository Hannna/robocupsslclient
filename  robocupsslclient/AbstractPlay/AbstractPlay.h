#ifndef ABSTRACTPLAN_H
#define ABSTRACTPLAN_H

class AbstractPlay
{
    public:
        AbstractPlay();
        void addTask();
        bool isFinished();
        virtual void execute()=0;
        virtual ~AbstractPlay();

    protected:
    private:
};

#endif // ABSTRACTPLAN_H
