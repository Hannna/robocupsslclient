/*
 * StartPlay.cpp
 *
 *  Created on: 24-05-2011
 *      Author: maciek
 */

#include "StartPlay.h"
#include "../VideoServer/Videoserver.h"
#include "../Tactics/PositionToStart.h"

StartPlay::StartPlay(std::string teamColor): Play(teamColor)  {

	//this->role0.addTactic( new ShootTactic( role0->getRobot() ) );

	//this->role1.addTactic( new ShootTactic( role1->getRobot() ) );

	//this->role2.addTactic( new ShootTactic( role2->getRobot() ) );

}

void StartPlay::execute(){
	LOG_INFO(log,"STARTING prepareForStart  " <<this->teamColor);
	Pose target;
	Pose p;
	if( this->teamColor.compare("red") == 0){
		if( Videoserver::redGoal == top )
			p = Pose(1,1,0);
		else
			p = Pose(-1,-1,0);
	}
	else{
		if( Videoserver::blueGoal == top )
			p = Pose(1,1,0);
		else
			p = Pose(-1,-1,0);
	}

	Pose robot0GoalPose( this->appConfig.field.FIELD_MIDDLE_POSE + p );
	this->role0.addTactic( new PositionToStart( robot0GoalPose, role0.getRobot( ) ) );

	if( this->teamColor.compare("red") == 0){
		if( Videoserver::redGoal == top )
			p = Pose (-1,1,0);
		else
			p = Pose(1,-1,0);
	}
	else{
		if( Videoserver::blueGoal == top )
			p = Pose (-1,1,0);
		else
			p = Pose(1,-1,0);
	}

	Pose robot1GoalPose( this->appConfig.field.FIELD_MIDDLE_POSE + p );
	this->role1.addTactic( new PositionToStart( robot1GoalPose, role1.getRobot( ) ) );

	if( this->teamColor.compare("red") == 0){
		if( Videoserver::redGoal == top ){
			p = Pose (0,-1,0);
		}
		else{
			p = Pose(0,1,0);
		}
	}
	else{
		if( Videoserver::blueGoal == top ){
			p = Pose (0,-1,0);
		}
		else{
			p = Pose(0,1,0);
		}
	}

	if( this->teamColor.compare("red") == 0){
		target = Pose( Videoserver::getRedGoalMidPosition(),0) ;
		std::cout<<" target "<< target <<"red  from video"<<Videoserver::getRedGoalMidPosition()<<std::endl;
	}
	else{
		target = Pose( Videoserver::getBlueGoalMidPosition(),0) ;
		std::cout<< " target "<< target <<"blue  from video"<<Videoserver::getBlueGoalMidPosition()<<std::endl;
	}

	Pose robot2GoalPose( target + p );
	std::cout<<" robot2GoalPose "<< robot2GoalPose << " target "<< target <<std::endl;

	//exit(0);
	this->role2.addTactic( new PositionToStart( robot2GoalPose, role2.getRobot( ) ) );


	this->role0.execute();
	this->role1.execute();
	this->role2.execute();

//	std::list<AbstractTactic *>::iterator tactic = tactics.begin();

//	for( ; tactic!= tactics.end();tactic++  ){
//		(*tactic)->start(NULL);
//	}

/*
	if( Videoserver::blueGoal == top )
		p = Pose(1,1,0);
	else
		p = Pose(-1,-1,0);

	Pose blue0GoalPose( this->appConfig.field.FIELD_MIDDLE_POSE + p );
	AbstractTactic * blue0Task =  new PositionToStart( blue0GoalPose,blue0);
	tactics.push_back(blue0Task);

	if( Videoserver::blueGoal == top )
		p = Pose (-1,1,0);
	else
		p = Pose(1,-1,0);

	Pose blue1GoalPose( this->appConfig.field.FIELD_MIDDLE_POSE + p );
	AbstractTactic * blue1Task =  new PositionToStart( blue1GoalPose,blue1);
	tactics.push_back(blue1Task);

	if( Videoserver::blueGoal == top )
		p = Pose (0,-1,0);
	else
		p = Pose(0,1,0);

	Pose blue2GoalPose( Pose( Videoserver::getBlueGoalMidPosition(),0 ) + p );
	AbstractTactic * blue2Task =  new PositionToStart( blue2GoalPose,blue2);
	tactics.push_back(blue2Task);
*/
}

void StartPlay::stop(){

}

void StartPlay::waitForFinish( ){

}

void StartPlay::reset(){

}

StartPlay::~StartPlay() {
	// TODO Auto-generated destructor stub
}
