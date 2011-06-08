/*
 * GoToPose.cpp
 *
 *  Created on: 2010-06-07
 *      Author: maciek
 */

#include "GoToPose.h"
#include "KickBall.h"
#include "../RRT/RRTPlanner.h"
#include "../Exceptions/SimulationException.h"

GoToPose::GoToPose(const Pose & pose,Robot * robot):Task(robot),goalPose(pose),serialize( Config::getInstance().isDebugMode() ) {
	this->rrt=NULL;
	currSimTime=0;
	lastSimTime=0;
	currSimTime=video.updateGameState(currGameState);
	force = false;
	LOG_INFO(log, "robot "<<robot->getRobotName()<<" create GoToPose Task, goto "<<this->goalPose<<" force "<<force );
}

GoToPose::GoToPose(const Pose & pose,Robot * robot, bool force_):Task(robot),goalPose(pose),serialize( Config::getInstance().isDebugMode() ) {
	this->rrt=NULL;
	currSimTime=0;
	lastSimTime=0;
	currSimTime=video.updateGameState(currGameState);
	force = force_;
	LOG_INFO(log, "robot "<<robot->getRobotName()<<" create GoToPose Task, goto "<<this->goalPose<<" force "<<force );
}


Task* GoToPose::nextTask(){

	//jesli robot ma pilke i warto oddac strzal to strzel
	if( this->evaluationModule.isRobotOwnedBall( this->robot ) ){
		//jesli ustawiono flage zezwalajaca na strzal
		if( (this->predicates & Task::kick_if_we_can) > 0){
			//oblicz czy wato strzelic na bramke
			std::pair<double,double> ang=evaluationModule.aimAtGoal( robot->getRobotName() );

			double score =
			( (ang.first * ang.second) > 0 ) ? fabs( ang.first + ang.second ) : fabs( ang.first) + fabs(ang.second );
			LOG_INFO(log, "current position score = "<<score<<" ang.first "<<ang.first<<" ang.second "<<ang.second );

			//jesli warto strzelic na bramke
			if( score > EvaluationModule::minOpenAngle ){
				LOG_INFO(this->log," GoToPose -> KickBall ");
				return new KickBall( robot, ( ang.first + ang.second )/2  ) ;
			}
		}
	}

	LOG_TRACE(log, "next TASK is null= " );

	return NULL;
}

/*
TaskSharedPtr & GoToPose::nextTask(){
	return TaskSharedPtr(this);
}
*/

Task::status GoToPose::run(void* arg, int steps){

	bool obsPredictionEnable=true;

	//pozycja do ktorej ma dojechac robot w kolejnym kroku
	Pose nextRobotPose;
	//biezaca pozycja robota
	Pose currRobotPose = (*currGameState).getRobotPos( robot->getRobotID() );

	//rotacja robota
	double robotRotation=0;

	LOG_DEBUG(log, "robot "<<robot->getRobotName()<<" run GoToPose Task, goto "<<this->goalPose <<"from "<< currRobotPose);

	//pozycja celu w ukladzie wsp zwiazanych z robotem
	Vector2D targetRelPosition;
	Vector2D robotCurrentGlobalVel;
	Vector2D robotNewGlobalVel;
	bool timeMeasure = false;

	static int serializedTrees = 0;

	/*
	 * za pomoca algorytmu rrt pokieruj robota do celu
	 */
	while( !this->stopTask && (steps--)!=0 ){

		currSimTime = video.updateGameState(currGameState) ;
		if( currSimTime <0 ){
			std::ostringstream s;
			s<<__FILE__<<":"<<__LINE__;
			throw SimulationException(s.str());
		}

		if( lastSimTime <  currSimTime ){
			lastSimTime = currSimTime;

            currRobotPose=(*currGameState).getRobotPos( robot->getRobotID() );
            RRTPlanner::ErrorCode status;

            if(rrt){
				delete rrt;
				if( this->force ){
					rrt = new RRTPlanner( Config::getInstance().getRRTGoalProb(),
							robot->getRobotName(),obsPredictionEnable,currGameState,goalPose,&path,currSimTime, timeMeasure, true );
				}
				else{
					rrt = new RRTPlanner( Config::getInstance().getRRTGoalProb(),
							robot->getRobotName(),obsPredictionEnable,currGameState,goalPose,&path,currSimTime, timeMeasure );
				}
				status=rrt->run( video.getUpdateDeltaTime() );
            }
            else{
            	if( this->force ){
            		rrt = new RRTPlanner( Config::getInstance().getRRTGoalProb(),
            							robot->getRobotName(),obsPredictionEnable,currGameState,goalPose,&path,currSimTime, timeMeasure, true );
            	}
            	else{
            		rrt = new RRTPlanner( Config::getInstance().getRRTGoalProb(),
                    						robot->getRobotName(),obsPredictionEnable,currGameState,goalPose,&path,currSimTime, timeMeasure );
            	}
            	status=rrt->run( video.getUpdateDeltaTime() );
            }

            if(serialize){
                std::string fileName("/home/maciek/codeblocks/magisterka/bin/Debug/");
                fileName.append(robot->getRobotName());
                fileName.append("_rrtTree.xml");
                rrt->serializeTree(fileName.c_str(),serializedTrees++);
            }

			if( status==RRTPlanner::Success ){

				GameStatePtr nextState=rrt->getNextState();

				if(nextState.get()==NULL){
					robot->setRelativeSpeed(Vector2D(0.0,0.0),0);
					LOG_DEBUG(log,"From rrtPlanner: next state is null. We arrive target");
					delete rrt;
					rrt = NULL;
					robot->stop();
					return Task::ok;
				}
				nextRobotPose=nextState->getRobotPos( robot->getRobotID() );

				robotRotation=currRobotPose.get<2>() ;
				//macierz obrotu os OY na wprost robota
				//RotationMatrix rmY(robotRotation);

				//pozycja celu w ukladzie wsp zwiazanych z robotem
				//targetRelPosition=rmY.Inverse()*(nextRobotPose.getPosition()-currRobotPose.getPosition());

				robotCurrentGlobalVel=(*currGameState).getRobotGlobalVelocity( robot->getRobotID() );

				//robotNewVel=calculateVelocity( robotCurrentVel, Pose(targetRelPosition.x,targetRelPosition.y,0));

				robotNewGlobalVel=calculateVelocity( robotCurrentGlobalVel, currRobotPose, nextRobotPose);
				//double w = robot->calculateAngularVel(*currGameState,robot->getRobotID(), goalPose);
				double w = robot->calculateAngularVel( currGameState->getRobotPos( robot->getRobotID() ), goalPose);
				//LOG_INFO(log,"move robot from"<<currRobotPose<<" to "<<nextRobotPose<<" robot curr global Vel"<<robotCurrentGlobalVel<<
				//		" setVel global vel "<<robotNewGlobalVel <<" w"<<w);

				//robot->setRelativeSpeed( robotNewVel, 0);
				//robot->setRelativeSpeed( robotNewVel, w );
				if( rrt->getDistToNearestObs() > 0.02 ){
					LOG_DEBUG(log,"move robot from"<<currRobotPose<<" to "<<nextRobotPose<<" robot curr global Vel"<<robotCurrentGlobalVel<<
											" setVel global vel "<<robotNewGlobalVel <<" w"<<w);

					robot->setGlobalSpeed(robotNewGlobalVel,w,robotRotation);

				}
				else{
					LOG_INFO(log,"move robot from"<<currRobotPose<<" to "<<nextRobotPose<<" robot curr global Vel"<<robotCurrentGlobalVel<<
											" setVel global vel "<<robotNewGlobalVel <<" w"<<0);

					robot->setGlobalSpeed(robotNewGlobalVel,0,robotRotation);
				}
				//robot->
			}
			else if(status==RRTPlanner::RobotReachGoalPose){
                LOG_DEBUG(log,"From rrtPlanner: RobotReachGoalPose");
                delete rrt;
                rrt = NULL;
                return Task::ok;
			}
			else{
				robot->setRelativeSpeed(Vector2D(0.0,0.0),0);
				LOG_WARN(log,"RRT to "<<this->goalPose<<" Error: "<<status);
				delete rrt;
				rrt = NULL;
				if(status==RRTPlanner::RobotCollision)
					return Task::collision;

				return Task::error;
			}
		}
	}

	if(this->stopTask)
		return Task::ok;

	return Task::not_completed;
}

GoToPose::~GoToPose() {
	delete this->rrt;
}
