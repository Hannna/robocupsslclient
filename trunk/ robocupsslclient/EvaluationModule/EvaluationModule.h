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
	enum ballState{
		//pilka jest wolna
		free,
		//pilka na aucie
		out,
		//pilka w bramce
		in_goal,
		//pilka zajmowana przez naszych
		occupied_our,
		//pilka zajmowana przez przeciwnika
		occupied_theirs
	};
    public:
     static EvaluationModule & getInstance();

     score aimAtGoal(const GameStatePtr & gameState, std::string robotName);
     /*@brief zwraca przedzial kata x=min;y=max
     */
     std::pair<double, double> aimAtGoal(const std::string& robotName);
     score aimAtTeamMate();

     ballState getBallState(Robot::robotID);

     Pose findBestDribbleTarget();

     bool haveBall_1( const Robot & robot);

     bool isRobotOwnedBall(const Robot & robot);
     bool isRobotOwnedBall(const Robot * robot);

     void test(Pose currRobotPose,Pose targetPosition);

     static const double minOpenAngle=0.52; //30 stopni
    private:
        static EvaluationModule * ptr;
        const Videoserver& video;
        Config & appConfig;
        static Mutex mutex;
        const log4cxx::LoggerPtr log;

        EvaluationModule();
        virtual ~EvaluationModule();
        inline EvaluationModule(const EvaluationModule &):video(Videoserver::getInstance()), appConfig(Config::getInstance()) {};
        inline EvaluationModule& operator=(const EvaluationModule &) { return *this; };
        /*@brief dla zadanej przeszkody znajduje przedział katowy z nia zwiazany
         *  jesli przeszkoda znajduje sie za robotem zwraca std::pair(-INF, -INF)
         */
        Set findObstacleCoverAngles(Pose currRobotPose,Pose obstaclePosition);
        void addToList(Set &set,std::list<Set> &sets);
};

#endif // EVALUATIONMODULE_H
