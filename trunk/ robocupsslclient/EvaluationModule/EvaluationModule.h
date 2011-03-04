#ifndef EVALUATIONMODULE_H
#define EVALUATIONMODULE_H
#include "../Lock/Lock.h"
#include "../additional.h"
#include "../VideoServer/Videoserver.h"
#include "../Robot/Robot.h"
#include "../Config/Config.h"

/*
    @brief Modul odpowiadajacy za ocene sytuacji na planszy
*/
namespace evaluation{

typedef double score;

}
class Set;

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
    private:
        static Mutex mutex;
        EvaluationModule();
        virtual ~EvaluationModule();
        inline EvaluationModule(const EvaluationModule &):video(Videoserver::getInstance()), appConfig(Config::getInstance()) {};
        inline EvaluationModule& operator=(const EvaluationModule &) { return *this; };
        /*@brief dla zadanej przeszkody znajduje przedział katowy z nia zwiazany
         *  jesli przeszkoda znajduje sie za robotem zwraca std::pair(-INF, -INF)
         */
        Set findObstacleCoverAngles(Pose currRobotPose,Pose obstaclePosition);
        void addToList(Set &set,std::list<Set> &sets);

        static EvaluationModule * ptr;
        const Videoserver& video;
    protected:
    private:
        Config & appConfig;
};

#endif // EVALUATIONMODULE_H
