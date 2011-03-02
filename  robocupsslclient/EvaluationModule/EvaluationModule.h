#ifndef EVALUATIONMODULE_H
#define EVALUATIONMODULE_H
#include "../Lock/Lock.h"
#include "../additional.h"
#include "../VideoServer/Videoserver.h"
#include "../Robot/Robot.h"


/*
    @brief Modul odpowiadajacy za ocene sytuacji na planszy
*/
namespace evaluation{

typedef double score;

}
using namespace evaluation;

class EvaluationModule
{
    public:
     static EvaluationModule & getInstance();
     score aimAtGoal(const GameStatePtr & gameState, std::string robotName);
     /*@brief zwraca przedzial kata x=min;y=max
     */
     std::pair<double, double> aimAtGoal(const std::string& robotName);
     score aimAtTeamMate();
     Pose findBestDribbleTarget();
     bool haveBall_1( const Robot & robot);
     bool haveBall_2(const Robot & robot);
     void test(Pose currRobotPose,Pose targetPosition);
    //private:
    public:
        static Mutex mutex;
        EvaluationModule();
        virtual ~EvaluationModule();
        inline EvaluationModule(const EvaluationModule &):video(Videoserver::getInstance() ) {};
        inline EvaluationModule& operator=(const EvaluationModule &) { return *this; };
        /*@brief dla zadanej przeszkody znajduje przedział katowy z nia zwiazany
         *
         */
        std::pair<double, double> findObstacleCoverAngles(Pose currRobotPose,Pose obstaclePosition);

        static EvaluationModule * ptr;
        const Videoserver& video;
    protected:
    private:
};

#endif // EVALUATIONMODULE_H
