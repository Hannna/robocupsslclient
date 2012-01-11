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

GoToPose::GoToPose(const Vector2D & position,Robot * robot, double maxDistToGoal_):
	Task(robot),goalPose(position),serialize( Config::getInstance().isDebugMode() ), maxDistToGoal( maxDistToGoal_), rotation(0),spec_rot(false),force(false)  {
	this->rrt=NULL;
	currSimTime=0;
	lastSimTime=0;
	//currSimTime=video.updateGameState(currGameState);

	xConstraints = NULL;
	yConstraints = NULL;

	LOG_INFO(log, "robot "<<robot->getRobotName()<<" create GoToPose Task, goto "<<this->goalPose<<" force "<<force<<" pointer "<<this );
	//do testow rrt
	total_rrt_time = 0;
	rrt_iterations = 0;
	max_path_size = 0;
	min_path_size = INT_MAX;
	max_tree_size = 0;
	min_tree_size = INT_MAX;
}

GoToPose::GoToPose( const Vector2D & position, const double rotation_,  Robot * robot, double maxDistToGoal_ ):
		Task(robot),goalPose(position),serialize( Config::getInstance().isDebugMode() ), maxDistToGoal( maxDistToGoal_),rotation(rotation_),spec_rot(true), force(false)  {

	this->rrt=NULL;
	currSimTime=0;
	lastSimTime=0;
	//currSimTime=video.updateGameState(currGameState);
	//force = false;

	xConstraints = NULL;
	yConstraints = NULL;
	//do testow rrt
	total_rrt_time = 0;
	rrt_iterations = 0;
	max_path_size = 0;
	min_path_size = INT_MAX;
	max_tree_size = 0;
	min_tree_size = INT_MAX;

	LOG_INFO(log, "robot "<<robot->getRobotName()<<" create GoToPose Task, goto "<<this->goalPose<<" rotation "<<rotation<<" force "<<force<<" pointer "<<this );

}
GoToPose::GoToPose(const Vector2D & position,Robot * robot, bool force_, double maxDistToGoal_):
		Task(robot),goalPose(position),serialize( Config::getInstance().isDebugMode() ), maxDistToGoal( maxDistToGoal_),rotation(0),spec_rot(false),force(force_) {
	this->rrt=NULL;
	currSimTime=0;
	lastSimTime=0;
	//currSimTime=video.updateGameState(currGameState);

	xConstraints = NULL;
	yConstraints = NULL;

	way_points_disabled = false;

	//do testow rrt
	total_rrt_time = 0;
	rrt_iterations = 0;
	max_path_size = 0;
	min_path_size = INT_MAX;
	max_tree_size = 0;
	min_tree_size = INT_MAX;

	LOG_INFO(log, "robot "<<robot->getRobotName()<<" create GoToPose Task, goto "<<this->goalPose<<" force "<<force<<" pointer "<<this );
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


Task::status GoToPose::run(void* arg, int steps){
	bool obsPredictionEnable=true;

	//pozycja do ktorej ma dojechac robot w kolejnym kroku
	Pose nextRobotPose;

	//rotacja robota
	double robotRotation=0;
	video.updateGameState(currGameState);
	LOG_INFO(log, "robot "<<robot->getRobotName()<<" run GoToPose Task, goto "<<this->goalPose <<"from "<< (*currGameState).getRobotPos( robot->getRobotID() ));

	//pozycja celu w ukladzie wsp zwiazanych z robotem
	Vector2D targetRelPosition;
	Vector2D robotCurrentGlobalVel;
	Vector2D robotNewGlobalVel;
	/*
	bool timeMeasure = false;

	if( strncmp( robot->getRobotName().c_str(), "blue0", 5 ) ){
		timeMeasure = true;
	}
*/
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
		//biezaca pozycja robota
		Pose currRobotPose = (*currGameState).getRobotPos( robot->getRobotID() );

		if( lastSimTime <  currSimTime ){
			lastSimTime = currSimTime;

            currRobotPose=(*currGameState).getRobotPos( robot->getRobotID() );
            RRTPlanner::ErrorCode status;

            if(rrt){
				delete rrt;
				if( this->force ){
					rrt = new RRTPlanner( Config::getInstance().getRRTGoalProb(),Config::getInstance().getRRTWayPointProb(),
							robot->getRobotName(),obsPredictionEnable,currGameState,
							Config::getInstance().getRRTMaxNodeNr(),Pose(goalPose,0),&path,currSimTime, true );
				}
				else{
					rrt = new RRTPlanner( Config::getInstance().getRRTGoalProb(),Config::getInstance().getRRTWayPointProb(),
							robot->getRobotName(),obsPredictionEnable,currGameState,
							Config::getInstance().getRRTMaxNodeNr(),Pose(goalPose,0),&path,currSimTime );
				}

				rrt->setMinDistance( maxDistToGoal );

				if( this->xConstraints ){
					rrt->addXConstraint( this->xConstraints );
				}

				if( this->yConstraints ){
					rrt->addYConstraint( this->yConstraints );
				}

				status=rrt->run( video.getUpdateDeltaTime() );
            }
            else{
            	if( this->force ){
            		rrt = new RRTPlanner( Config::getInstance().getRRTGoalProb(),Config::getInstance().getRRTWayPointProb(),
            							robot->getRobotName(),obsPredictionEnable,currGameState,
            							Config::getInstance().getRRTMaxNodeNr(),Pose(goalPose,0),&path,currSimTime, true );
            	}
            	else{
            		rrt = new RRTPlanner( Config::getInstance().getRRTGoalProb(), Config::getInstance().getRRTWayPointProb(),
                    						robot->getRobotName(),obsPredictionEnable,currGameState,
                    						Config::getInstance().getRRTMaxNodeNr(),Pose(goalPose,0),&path,currSimTime );
            	}
            	rrt->setMinDistance( maxDistToGoal );
            	status=rrt->run( video.getUpdateDeltaTime() );

            	//if( status == )
            }

        	//do testow rrt
            if( path.size() > 0 || rrt->getTreeSize() > 0){
            	rrt_iterations+=1;
            	total_rrt_time+=rrt->getRRTTime();
            }

            if( path.size() > max_path_size )
            	max_path_size = path.size();

            if( ( path.size() < min_path_size ) && ( path.size() > 0 ) )
                min_path_size = path.size();

            if( rrt->getTreeSize() > max_tree_size )
            	max_tree_size = rrt->getTreeSize();

            if( ( rrt->getTreeSize() < min_tree_size ) && ( rrt->getTreeSize() > 0 ) )
                min_tree_size = rrt->getTreeSize();


            if(serialize){
                std::string fileName("");
                fileName.append(robot->getRobotName());
                fileName.append("_rrtTree.xml");
                rrt->serializeTree(fileName.c_str(),serializedTrees++);
            }

            if( this->way_points_disabled ){
            	path.clear();
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
				bool haveBall = this->evaluationModule.isRobotOwnedBall( this->robot );
				double w = 0;

				robotNewGlobalVel=calculateVelocity( robotCurrentGlobalVel, currRobotPose, nextRobotPose);

				// odleglosc punktu startowego od najblizszej przeszkody, brane sa pod uwage jedynie biezace polozenia robotow, jesli robot jest blizej przeszkody
				// to wywolywana jest funkcja powodujaca rozproszenie
				if( rrt->getDistToNearestObs() > GoToPose::minDistFromObstacle ){
					//if( this->predicates && Task::got_ball ){
					/*jesli mam pilke to sprawdz czy:
					 * 1. robot jest zworcony przodem do punktu docelowego
					 * 2. jesli nie jest to wykonaj obrot robota wokol pillki
					 *
					 */

					if( haveBall ){


						LOG_FATAL( log, "####################################################################################################################");

						//while( haveBall && ( fabs(t.get<0>()) > 0.3 || t.get<1>() < 0 ) ){

						double currAlfaToCel=10;
						/*
						while( haveBall && fabs( currAlfaToCel ) > M_PI/2.0 ){
						// ten kawalek kodu wyznacza kat o jaki robot musi sie obrocic zeby byc skierowanym na cel
							RotationMatrix rm0(0);
							//Pose ballPose( ballPos,0 );

							Pose reltargetPose_ = nextRobotPose.transform( currRobotPose.getPosition(),rm0 );
							Pose reltargetPose = reltargetPose_*100;
							double rotacjaDocelowa=-atan2(reltargetPose.get<0>(),reltargetPose.get<1>()) ;

							assert( fabs(rotacjaDocelowa) < M_PI);

							//obrot jaki trzeba wykonac w biezacym kroku
							currAlfaToCel = convertAnglePI( rotacjaDocelowa - robotRotation );
							double maxW = fabs(robotRotation)/video.getUpdateDeltaTime() > M_PI ? M_PI : fabs(currAlfaToCel)/video.getUpdateDeltaTime() ;
							//double angularVel=Ko*( convertAnglePI(rotacjaDocelowa-currGlobalRobotRot) )+ Ker*( convertAnglePI(oldAlfaToCel - currAlfaToCel) );

							//oldAlfaToCel=currAlfaToCel;
							robot->setLastTetaToBall(currAlfaToCel);
							//double w = fabs(currAlfaToCel) > M_PI/2 ? M_PI/2 * sgn(currAlfaToCel) : currAlfaToCel;


							LOG_FATAL( log, "currRobotPose "<< currRobotPose <<" globalNextPose "<<nextRobotPose );
							//double maxW = fabs(angle)/video.getUpdateDeltaTime() > M_PI ? M_PI : fabs(angle)/video.getUpdateDeltaTime() ;
							double ball_radious = 0.02;

							//double angle = t.getPosition().angleTo( Vector2D( 0.0,1.0 ) );

							boost::tuple< double, double, double > vel = calculateCurwatureVelocity( ball_radious*sgn(currAlfaToCel) , maxW );
							Vector2D v = Vector2D( vel.get<0>(), vel.get<1>() );
							double w = vel.get<2>();
							robot->setRelativeSpeed( v, w );
							while( lastSimTime - currSimTime >= 0 ){
								currSimTime = video.updateGameState(currGameState);
							}
							lastSimTime = currSimTime;
							//currSimTime = video.updateGameState(currGameState) ;
							currRobotPose=(*currGameState).getRobotPos( robot->getRobotID() );
							robotRotation = currRobotPose.get<2>();
							//rm = RotationMatrix(robotRotation);
							//t = nextRobotPose.transform( currRobotPose.getPosition() , rm);
							haveBall = this->evaluationModule.isRobotOwnedBall( this->robot );
						}*/
						//zakomentowano 30 12 2011
						//macierz obrotu os OY na wprost robota
						RotationMatrix rm(robotRotation);
						Pose t = nextRobotPose.transform( currRobotPose.getPosition() , rm);

						while( haveBall && ( fabs(t.get<0>()) > 0.3 || t.get<1>() < 0 ) ){
						//while( haveBall && (  t.get<1>() < 0 ) ){
							//Vector2D oy( 0.0,1.0 );
							//double angle = oy.angleTo( t.getPosition( ) );
							double angle = convertAnglePI(atan2(t.get<1>(),t.get<0>()) -M_PI/2.0);
							LOG_FATAL( log, "currRobotPose "<< currRobotPose <<" globalNextPose "<<nextRobotPose << " relative nextRobotPose  "<<t<<" angle "<<angle );
							double maxW = fabs(angle)/video.getUpdateDeltaTime() > 2*M_PI ? 2*M_PI : fabs(angle)/video.getUpdateDeltaTime() ;
							double ball_radious = 0.02;

							//double angle = t.getPosition().angleTo( Vector2D( 0.0,1.0 ) );

							boost::tuple< double, double, double > vel = calculateCurwatureVelocity( ball_radious*sgn(angle) , maxW );
							Vector2D v = Vector2D( vel.get<0>(), vel.get<1>() );
							double w = vel.get<2>();
							robot->setRelativeSpeed( v, w );
							while( lastSimTime - currSimTime >= 0 ){
								currSimTime = video.updateGameState(currGameState);
							}
							lastSimTime = currSimTime;
							//currSimTime = video.updateGameState(currGameState) ;
							currRobotPose=(*currGameState).getRobotPos( robot->getRobotID() );
							robotRotation = currRobotPose.get<2>();
							rm = RotationMatrix(robotRotation);
							t = nextRobotPose.transform( currRobotPose.getPosition() , rm);
							haveBall = this->evaluationModule.isRobotOwnedBall( this->robot );

							LOG_FATAL( log, "haveBall "<<haveBall );
						}
						LOG_FATAL( log, "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
						robotCurrentGlobalVel=(*currGameState).getRobotGlobalVelocity( robot->getRobotID() );
						robotNewGlobalVel=calculateVelocity( robotCurrentGlobalVel, currRobotPose, nextRobotPose);

						//jesli robot mam miec szczegolna rotacje w punckie docelowym
						//if(this->spec_rot){
						//	w = robot->calculateAngularVel( currGameState->getRobotPos( robot->getRobotID() ), this->rotation, currGameState->getSimTime(), haveBall );
						//}
						//else
							w = robot->calculateAngularVel( currGameState->getRobotPos( robot->getRobotID() ), goalPose, currGameState->getSimTime(),haveBall );

						LOG_DEBUG(log,"move robot from"<<currRobotPose<<" to "<<nextRobotPose<<" robot curr global Vel"<<robotCurrentGlobalVel<<
																		" setVel global vel "<<robotNewGlobalVel <<" w"<<w);
						//if(fabs(w) >1.0 )
						//	robot->setGlobalSpeed(Vector2D(0.0,0.0),w,robotRotation);
						//else
							robot->setGlobalSpeed(robotNewGlobalVel,w,robotRotation);



						//double deltaW = currGameState->getRobotAngularVelocity( robot->getRobotID() );
						//deltaW -=w;
						//if( fabs(deltaW) > 0.5 ){
						//	deltaW = 0.5 * sgn(w);
						//	LOG_DEBUG(log,"move robot from"<<currRobotPose<<" to "<<nextRobotPose<<" robot curr global Vel"<<robotCurrentGlobalVel<<
						//												" setVel global vel "<<robotNewGlobalVel <<" w"<<w);
//
//							robot->setGlobalSpeed(robotNewGlobalVel,w,robotRotation);
//						}
//
//						else{
//							LOG_DEBUG(log,"move robot from"<<currRobotPose<<" to "<<nextRobotPose<<" robot curr global Vel"<<robotCurrentGlobalVel<<
//												" setVel global vel "<<robotNewGlobalVel <<" w"<<w);
//							robot->setGlobalSpeed(robotNewGlobalVel,w,robotRotation);
//						}


					}
					//else
					{
						if(this->spec_rot){
							w = robot->calculateAngularVel( currGameState->getRobotPos( robot->getRobotID() ), this->rotation, currGameState->getSimTime(), haveBall );
						}
						else
							w = robot->calculateAngularVel( currGameState->getRobotPos( robot->getRobotID() ), goalPose, currGameState->getSimTime(),haveBall );

						LOG_DEBUG(log,"move robot from"<<currRobotPose<<" to "<<nextRobotPose<<" robot curr global Vel"<<robotCurrentGlobalVel<<
												" setVel global vel "<<robotNewGlobalVel <<" w"<<w);
						//if(fabs(w) >1.0 )
						//	robot->setGlobalSpeed(Vector2D(0.0,0.0),w,robotRotation);
						//else
							robot->setGlobalSpeed(robotNewGlobalVel,w,robotRotation);
					}
				}
				else{
					LOG_FATAL(log,"disperse move robot from"<<currRobotPose<<" to "<<nextRobotPose<<" robot curr global Vel"<<robotCurrentGlobalVel<<
											" setVel global vel "<<robotNewGlobalVel <<" w"<<0);

					//robot->setGlobalSpeed(robotNewGlobalVel,0,robotRotation);
					robot->disperse( GoToPose::minDistFromObstacle*2.0 );
				}
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

	if(this->stopTask){
		LOG_DEBUG(log,"task was stopped");
		return Task::ok;
	}

	return Task::not_completed;
}

GoToPose::~GoToPose() {
	delete this->rrt;
	LOG_DEBUG(log,"pointer "<<this);
}
