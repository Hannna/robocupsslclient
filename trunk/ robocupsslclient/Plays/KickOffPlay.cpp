/*
 * KickOffPlay.cpp
 *
 *  Created on: 24-05-2011
 *      Author: maciek
 */

#include "KickOffPlay.h"

KickOffPlay::KickOffPlay( std::string teamColor ): Play( teamColor, 3) {
	/*przygotowuje roboty do wykopu pilki ze srodka boiska
	 *
	 */
//	void SimplePlay::prepareForKickOff(const Vector2D& kickoffPose){
/*
		LOG_INFO(log,"STARTING prepareForKickOff  " <<this->teamColor);
		Pose robot0GoalPose(kickoffPose,0);

	//	AbstractTactic * robot0Task =  new PositionToStart( robot0GoalPose,robot0);
		AbstractTactic * robot0Task =  new Pass(*robot0,robot1->getRobotID());
		robot0Task->markParam( AbstractTactic::start_from_kickoff );
		tactics.push_back( robot0Task );
		//robot0Task->start(NULL);


		Pose target;
		Pose p;
	/*
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
		AbstractTactic * robot0Task =  new PositionToStart( robot0GoalPose,robot0);
		tactics.push_back( robot0Task );
	*/
/*		if( this->teamColor.compare("red") == 0){
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
		AbstractTactic * robot1Task =  new PositionToStart( robot1GoalPose,robot1);
		tactics.push_back(robot1Task);

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
		}
		else{
			target = Pose( Videoserver::getBlueGoalMidPosition(),0) ;
		}

		Pose robot2GoalPose( target + p );
		AbstractTactic * robot2Task =  new PositionToStart( robot2GoalPose,robot2);
		tactics.push_back(robot2Task);

		std::list<AbstractTactic *>::iterator tactic = tactics.begin();

		for( ; tactic!= tactics.end();tactic++  ){
			(*tactic)->start(NULL);
		}

	}
*/
}

KickOffPlay::~KickOffPlay() {
	// TODO Auto-generated destructor stub
}
