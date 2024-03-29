#ifndef EVALUATIONMODULE_H
#define EVALUATIONMODULE_H
#include "../Lock/Lock.h"
#include "../additional.h"
#include "../VideoServer/Videoserver.h"
#include "../Robot/Robot.h"
#include "../Config/Config.h"
#include "../GameState/GameState.h"
#include "EvaluationTypes.h"
/*
    @brief Modul odpowiadajacy za ocene sytuacji na planszy
*/
namespace evaluation{
//okresla szanse powodzenia dango ruchu/akcji
//score zawiera się w [0,1] gdzie 0 oznacza brak powodzenia 1 pewny sukces
typedef double score;

}
class Set;

using namespace evaluation;

class EvaluationModule
{
public:

    public:
     static EvaluationModule & getInstance();

     score aimAtGoal(const GameStatePtr & gameState, std::string robotName) const;
     /*@brief zwraca przedzial kata x=min;y=max
     */
     std::pair<double, double> aimAtGoal(const std::string& robotName,double& angleToShoot,double & score) const;

     std::pair<double, double> aimAtGoal(const GameStatePtr & gameState,const std::string& robotName,double& angleToShoot,double & score) const;

     /*@brief zwraca przedzial kata x=min;y=max
      *
      */
     score aimAtTeamMate( Robot::robotID myRobotID, Robot::robotID goalRobotID, double * rotationToTarget = NULL );

     bool checkAngleToPass(Vector2D targetPosition, Pose currRobotPosition,double & angleToTarget) const;

     BallState::ballState getBallState(Robot::robotID, bool * iAmCloserToBall = NULL);

     BallState::ballState getBallState(Vector2D ballPosition);

     Pose getPositionForThrowIn(){
    	 return Pose(this->positionForThrowIn,0);
     }

    // Pose findBestDribbleTarget(const std::string robotName,Robot::robotID rid);
     Pose findBestDribbleTarget( Vector2D centerPoint, const std::string robotName, Robot::robotID rid );

     //bool haveBall_1( const Robot & robot);

     bool isRobotOwnedBall(const Robot::robotID & robotId);

     bool isRobotOwnedBall(const Robot & robot);

     bool isRobotOwnedBall(const Robot * robot);

     bool isRobotOwnedBall(const Robot & robot, const GameStatePtr& currGameState,double& distanceToBall, double& angleToBall);

     bool isRobotOwnedBall(const Robot::robotID & robotID, const GameStatePtr& currGameState,double& distanceToBall, double& angleToBall);

     void test(Pose currRobotPose,Pose targetPosition);

     static const double minOpenAngle=0.33; //30 stopni
    private:
        static EvaluationModule * ptr;
        const Videoserver& video;
        Config & appConfig;
        static Mutex mutex;
        const log4cxx::LoggerPtr log;
        Vector2D positionForThrowIn;
        BallState::ballState ballState_;

        EvaluationModule();
        virtual ~EvaluationModule();
        inline EvaluationModule(const EvaluationModule &):video(Videoserver::getInstance()), appConfig(Config::getInstance()) {};
        inline EvaluationModule& operator=(const EvaluationModule &) { return *this; };
        /*@brief dla zadanej przeszkody znajduje przedział katowy z nia zwiazany
         *  jesli przeszkoda znajduje sie za robotem zwraca std::pair(-INF, -INF)
         */
        Set findObstacleCoverAngles(Pose currRobotPose,Pose obstaclePosition, double rotation) const;
        bool addToList(Set &set,std::list<Set> &sets) const;
};



#endif // EVALUATIONMODULE_H
