#include "ShootTactic.h"
#include "../EvaluationModule/EvaluationModule.h"
#include "../Task/Task.h"
#include "../Task/GoToPose.h"
#include "../Task/KickBall.h"



ShootTactic::ShootTactic(Robot & robot): AbstractTactic(robot)
{

}


void ShootTactic::execute(void *){

    EvaluationModule& evaluation=EvaluationModule::getInstance();
    std::pair<double, double> ang=evaluation.aimAtGoal(robot.getRobotName());

    LOG_DEBUG(log, "##########################################  angmin "<<ang.first<<" angmax"<<ang.second );

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
    		    		Config::getInstance().field.TOP_GOAL_MID_POSITION.y, 0);
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
    int steps=10;
    bool result;
    double score_ ;
    //jesli kat do strzalu jest mniejszy niz 30 stopni
    //rzez 10 krokow jedz do bramki
    do{
       this->currentTask = TaskPtr( new GoToPose( goalPose,&robot) );
       result = this->currentTask->execute(NULL, steps);

       if(!result)
    	   return;

        ang=evaluation.aimAtGoal(robot.getRobotName());
        LOG_DEBUG(log, "angmin "<<ang.first<<" angmax"<<ang.second );

        score_ = fabs( ang.first - ang.second );

        goalPose.get<2>() = ( ang.first + ang.second )/2 ;

        LOG_DEBUG(log, "current position score = "<<score_<<" rotation to goal= goalPose.get<2>()"<<goalPose.get<2>() );

    }while( score_ < EvaluationModule::minOpenAngle );

	this->currentTask->stop();


	this->currentTask = TaskPtr( new KickBall(&robot, goalPose.get<2>() ) );
	this->currentTask->execute(NULL);

	this->currentTask->stop();

/*
    //sprawdz czy dany robot ma pilke
    if(evaluation.haveBall_1(robot)){
        std::pair<double, double> ang=evaluation.aimAtGoal(robot.getRobotName());
        if( fabs(ang.first - ang.second ) > M_PI/6.0 ){
            robot.kick();
        }
    }
    else{
    //gotoballTask
    ;
    }
*/

}

bool  ShootTactic::isFinish(){

    return true;
}

ShootTactic::~ShootTactic()
{

}
