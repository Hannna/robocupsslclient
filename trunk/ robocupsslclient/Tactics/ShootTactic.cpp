#include "ShootTactic.h"
#include "../EvaluationModule/EvaluationModule.h"
#include "../Task/Task.h"
#include "../Task/GoToPose.h"
#include "../Task/KickBall.h"
#include "../Task/GoToBall.h"
#include "../Task/MoveBall.h"




ShootTactic::ShootTactic(Robot & robot): AbstractTactic(robot)
{

}


void ShootTactic::execute(void *){

    EvaluationModule& evaluation=EvaluationModule::getInstance();
    std::pair<double, double> ang=evaluation.aimAtGoal(robot.getRobotName());

    Pose goalPose;
    //TODO: sprawdzic rotacje inna powinna byc w przypadku jazdy na bramke dolna
    //a inna w przypadku jazdy na gorna
    if(robot.getRobotName().compare(0,3,"red")==0){
    	if(Videoserver::redGoal==bottom){
    		goalPose = Pose(Config::getInstance().field.BOTTOM_GOAL_MID_POSITION.x,
    		    		Config::getInstance().field.BOTTOM_GOAL_MID_POSITION.y, 0.0 );
    	}
    	else{
            goalPose = Pose(Config::getInstance().field.TOP_GOAL_MID_POSITION.x,
    		    		Config::getInstance().field.TOP_GOAL_MID_POSITION.y - 0.01, 0);
    	}
    }
    else{
    	if(Videoserver::blueGoal==bottom){
    		goalPose = Pose(Config::getInstance().field.BOTTOM_GOAL_MID_POSITION.x,
    		    		Config::getInstance().field.BOTTOM_GOAL_MID_POSITION.y, 0);
    	}
    	else{
    		goalPose = Pose(Config::getInstance().field.TOP_GOAL_MID_POSITION.x,
    		    		Config::getInstance().field.TOP_GOAL_MID_POSITION.y, 0);
    	}
    }

    goalPose.get<2>() = ( ang.first + ang.second )/2 ;

    Task::status taskStatus = Task::not_completed;
    double score =0;
    double bestScore = 0;

    //(score, target) ← evaluation.aimAtGoal()
    //if (was kicking at goal) then
    //score ← score + HYSTERESIS
    // SParami ← setCommand(MoveBall, target, KICK IF WE CAN)

    while(true){
    	taskStatus = Task::not_completed;
		this->currentTask = TaskSharedPtr( new MoveBall( goalPose, &robot ) );
		this->currentTask->markParam(Task::kick_if_we_can);
		bestScore = score;
		Task* newTask;
		while(taskStatus!=Task::ok){
			newTask = this->currentTask->nextTask();

			if(newTask){
				this->currentTask = TaskSharedPtr( newTask );
			}
			int steps=1;
			taskStatus = this->currentTask->execute(NULL,steps);

			if( taskStatus == Task::error ){
				LOG_ERROR(log,"Shoot tactic Task::error " <<taskStatus );
				break;
			}

			if( taskStatus == Task::collision ){
				robot.stop();
				LOG_FATAL(log,"Shoot tactic Task::error " <<taskStatus );
				return;
			}
		}
		//Nie jest potrzebne bo wskaznik jest przechowywany w smartPtr currentTask
		//if(newTask)
		//	delete newTask;
    }
}

bool  ShootTactic::isFinish(){

    return true;
}

ShootTactic::~ShootTactic()
{

}
