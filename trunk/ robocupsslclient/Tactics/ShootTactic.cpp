#include "ShootTactic.h"
#include "../EvaluationModule/EvaluationModule.h"
#include "../Task/Task.h"
#include "../Task/GoToPose.h"
#include "../Task/KickBall.h"
#include "../Task/GoToBall.h"
#include "../Task/MoveBall.h"

ShootTactic::ShootTactic(Robot & robot): Tactic(robot)
{
	this->active = true;
}


void ShootTactic::execute(void *){

	LOG_INFO(log,"Start Shoot tactic " );
    EvaluationModule& evaluation=EvaluationModule::getInstance();
    double angleToGoal=0;
    //std::pair<double, double> ang=evaluation.aimAtGoal(robot.getRobotName(),angleToGoal);

    //Pose goalPose  = evaluation.findBestDribbleTarget(robot.getRobotName(), robot.getRobotID());
    //TODO: sprawdzic rotacje inna powinna byc w przypadku jazdy na bramke dolna
    //a inna w przypadku jazdy na gorna
    /*
    if(robot.getRobotName().compare(0,3,"red")==0){
    	goalPose = Pose( Videoserver::getBlueGoalMidPosition().x,
    						Videoserver::getBlueGoalMidPosition().y, 0.0 );
    }
    else{
    	goalPose = Pose( Videoserver::getRedGoalMidPosition().x,
    	    		Videoserver::getRedGoalMidPosition().y, 0.0 );
    }
*/

	//LOG_INFO(log,"angle to goal "<<ang.first<<" "<<ang.second );


	//jesli warto strzelic na bramke
	//if( fabs(score) > EvaluationModule::minOpenAngle   ){
	//	LOG_INFO(this->log,"MoveBall -> KickBall score "<<score<<"  ang.first"<<ang.first<<" ang.second "<<ang.second );
	//	return new KickBall( robot,  convertAnglePI( ang.first + sign*score/2.0 )  ) ;
	//}

    ////




	//goalPose = Pose( goalPose.get<0>(), goalPose.get<1>(), angleToGoal  );
	//goalPose.get<2>() = angleToGoal;

    Task::status taskStatus = Task::not_completed;
    //double score =0;
    //score =0;
    //double bestScore = 0;

    //(score, target) ← evaluation.aimAtGoal()
    //if (was kicking at goal) then
    //score ← score + HYSTERESIS
    // SParami ← setCommand(MoveBall, target, KICK IF WE CAN)

    Task::predicate predicates = Task::null;
    while(true){
        {
     	LockGuard l(this->mutex);
     	if(this->stop)
     		break;
        }

    	LOG_INFO(log,"start calculate best dribble target " );
    	Vector2D centerPosition(2.7, 1.875);
    	//(Config::getInstance().field.BOTTOM_GOAL_MID_POSITION
    	//Config::getInstance().field.BOTTOM_GOAL_MID_POSITION
    	BallState::ballState bs = evaluation.getBallState(Robot::blue);
    	if(bs == BallState::in_goal)
    		break;

    	Pose goalPose  = evaluation.findBestDribbleTarget(centerPosition,
    			robot.getRobotName(), robot.getRobotID());
    	LOG_INFO(log,"end calculate best dribble target " );
    	taskStatus = Task::not_completed;
		this->currentTask = TaskSharedPtr( new MoveBall( goalPose, &robot ) );
		Task::predicate p = Task::kick_if_we_can;
		this->currentTask->markParam(p);
		this->currentTask->markParam(predicates);
		//bestScore = score;
		Task* newTask;
		int steps=1;
		while( taskStatus!=Task::ok && !this->stop ){
			newTask = this->currentTask->nextTask();

			if( newTask ){
				this->currentTask = TaskSharedPtr( newTask );
			}

			taskStatus = this->currentTask->execute(NULL,steps);

			if( taskStatus == Task::error ){
				LOG_ERROR(log,"Shoot tactic Task::error " <<taskStatus );
				robot.stop();
				break;
			}

			if( taskStatus == Task::collision ){
				robot.stop();
				LOG_FATAL(log,"Shoot tactic Task::collision. exit from tactic " );
				break;
				//LockGuard m(mutex);
				//finished = true;
				//return;
			}

			if( taskStatus == Task::kick_ok ){
				robot.stop();
				LOG_FATAL(log," Shoot tactic Task::kick_ok " );

				taskStatus = Task::ok;
				while( 1 ){
					BallState::ballState ballState_  = evaluation.getBallState( robot.getRobotID() );
					if( ballState_ == BallState::go_to_goal ){
						usleep(10000);
					}
					else if( ballState_ == BallState::in_goal ){
						LockGuard m(mutex);
						finished = true;
						LOG_INFO(log,"exit from shoot tactic " );
						 pthread_cond_broadcast(&this->finish_cv);
						return;
					}
					else
						break;
				}
			}

			if( taskStatus == Task::get_ball ){
				taskStatus = Task::ok;
				predicates = Task::got_ball;
				//robot.stop();
				//LOG_FATAL(log," Shoot tactic Task::kick_ok " );_
				//return;
			}

		}
		//Nie jest potrzebne bo wskaznik jest przechowywany w smartPtr currentTask
		//if(newTask)
		//	delete newTask;
    }
    LockGuard m(mutex);
    finished = true;
    pthread_cond_broadcast(&this->finish_cv);
    LOG_INFO(log,"exit from shoot tactic " );
}

bool  ShootTactic::isFinish(){
	LockGuard m(mutex);
    return this->finished;
}

ShootTactic::~ShootTactic()
{

}
