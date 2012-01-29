#include "ShootTactic.h"
#include "../EvaluationModule/EvaluationModule.h"
#include "../Task/Task.h"
#include "../Task/GoToPose.h"
#include "../Task/KickBall.h"
#include "../Task/GoToBall.h"
#include "../Task/MoveBall.h"




ShootTactic::ShootTactic(Robot & robot): Tactic(robot)
{

}


void ShootTactic::execute(void *){

	LOG_INFO(log,"Start Shoot tactic " );
    EvaluationModule& evaluation=EvaluationModule::getInstance();
    double angleToGoal=0;
    std::pair<double, double> ang=evaluation.aimAtGoal(robot.getRobotName(),angleToGoal);

    Pose goalPose;
    //TODO: sprawdzic rotacje inna powinna byc w przypadku jazdy na bramke dolna
    //a inna w przypadku jazdy na gorna
    if(robot.getRobotName().compare(0,3,"red")==0){
    	goalPose = Pose( Videoserver::getBlueGoalMidPosition().x,
    						Videoserver::getBlueGoalMidPosition().y, 0.0 );
    }
    else{
    	goalPose = Pose( Videoserver::getRedGoalMidPosition().x,
    	    		Videoserver::getRedGoalMidPosition().y, 0.0 );
    }


	LOG_INFO(log,"angle to goal "<<ang.first<<" "<<ang.second );


	//jesli warto strzelic na bramke
	//if( fabs(score) > EvaluationModule::minOpenAngle   ){
	//	LOG_INFO(this->log,"MoveBall -> KickBall score "<<score<<"  ang.first"<<ang.first<<" ang.second "<<ang.second );
	//	return new KickBall( robot,  convertAnglePI( ang.first + sign*score/2.0 )  ) ;
	//}

    ////




	goalPose = Pose( goalPose.get<0>(), goalPose.get<1>(), angleToGoal  );
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
    while( !this->stop ){
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
				LOG_FATAL(log,"Shoot tactic Task::collision " );
				finished = true;
				return;
			}

			if( taskStatus == Task::kick_ok ){
				robot.stop();
				LOG_FATAL(log," Shoot tactic Task::kick_ok " );
				finished = true;
				return;
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
    finished = true;
    LOG_INFO(log,"exit from shoot tactic " );
}

bool  ShootTactic::isFinish(){

    return this->finished;
}

ShootTactic::~ShootTactic()
{

}
