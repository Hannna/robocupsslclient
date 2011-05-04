/*
 * RRTPlanner.cpp
 *
 *  Created on: 2009-12-17
 *      Author: Maciej Gąbka
 */

#include "RRTPlanner.h"

#include <algorithm>
#include <limits>
#include <sstream>
#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <boost/foreach.hpp>
#include <boost/bind.hpp>

#define MY_ENCODING "ISO-8859-1"
#include <libxml/xmlsave.h>
#include <libxml/xmlmemory.h>
#include <libxml/parserInternals.h>
#include <libxml/tree.h>
#include <errno.h>


#include "../Logger/Logger.h"
#include "../RotationMatrix/RotationMatrix.h"

//zasieg w jakim losujemy cel
const double RRTPlanner::randomStateReach=0.5;//[m]

//generatory liczb losowych do losowania pozycji w drzewie rrt
static boost::mt19937 rngA(static_cast<unsigned> (time(NULL)));
static boost::uniform_01<boost::mt19937> uniformGen_01(rngA);

/*
const double RRTPlanner::maxXvalue ( Config::getInstance().field.FIELD_TOP_RIGHT_CORNER.x - Config::getInstance().field.FIELD_MARIGIN );
const double RRTPlanner::minXvalue ( Config::getInstance().field.FIELD_BOTTOM_LEFT_CORNER.x + Config::getInstance().field.FIELD_MARIGIN );
const double RRTPlanner::maxYvalue ( Config::getInstance().field.FIELD_TOP_RIGHT_CORNER.y - Config::getInstance().field.FIELD_MARIGIN );
const double RRTPlanner::minYvalue ( Config::getInstance().field.FIELD_BOTTOM_LEFT_CORNER.y + Config::getInstance().field.FIELD_MARIGIN );
*/

RRTPlanner::RRTPlanner(const double goalProb,const std::string robotName_,bool withObsPrediction,
			const GameStatePtr currState,const Pose goalPose_,std::list<Pose> * path, double simTime_, bool timeMeasure):
				robotName(robotName_ ),
				robotId(Robot::getRobotID(robotName_)),
                root( new RRTNode( currState,robotId ) ),
                goalPose( goalPose_ ),
                obsPredictionEnabled( withObsPrediction ),
                toTargetLikelihood( goalProb ),
                simTime( simTime_ ),
                logger( getLoggerPtr(robotName_.c_str()) ),
                maxXvalue ( Config::getInstance().field.FIELD_TOP_RIGHT_CORNER.x - Config::getInstance().field.FIELD_MARIGIN ),
                minXvalue ( Config::getInstance().field.FIELD_BOTTOM_LEFT_CORNER.x + Config::getInstance().field.FIELD_MARIGIN ),
                maxYvalue ( Config::getInstance().field.FIELD_TOP_RIGHT_CORNER.y - Config::getInstance().field.FIELD_MARIGIN ),
                minYvalue ( Config::getInstance().field.FIELD_BOTTOM_LEFT_CORNER.y + Config::getInstance().field.FIELD_MARIGIN )
    {

//	LOG_TRACE( logger,"maxXvalue "<<maxXvalue<<" minXvalue "<<minXvalue<<" maxYvalue "<<maxYvalue<<" minYvalue "<<minYvalue );


    static int rrtNr;

    goDirectToTarget=false;
    finish=false;

	//clock_t startTime=clock();
	//clock_t currTime;
	//long long int clocks_per_msec=CLOCKS_PER_SEC/1000;
	//std::cout<<"CLOCKS_PER_SEC"<<CLOCKS_PER_SEC<<"clocks_per_msec "<<clocks_per_msec<<std::endl;
	//std::cout<<"startTime"<<startTime<<std::endl;

	this->path=path;
	int i=0;
	if(this->path->size()>0){
		std::list<Pose>::iterator ii=this->path->begin();
		Pose tmp=*ii;
		wayPoints.push_back(tmp);
		for(; ii!=this->path->end();ii++){
			if( tmp.distance(*ii) > 0.05 ){
				tmp=*ii;
				i++;
				wayPoints.push_back(tmp);
			}

		}
	}

    LOG_DEBUG( logger," creating rrt nr "<<rrtNr++<<" simTime "<<simTime<<"  goal Pose "<<goalPose
    		<<" and currPose "<<currState->getRobotPos(robotId)
    		<<" use "<<i<<" way points" );

    root->setTargetPose(this->goalPose);
}

RRTPlanner::ErrorCode RRTPlanner::run(double deltaSimTime){

	this->foundNewPlan = false;

	LOG_TRACE(logger,"start run");
    struct timespec startTime;
    measureTime(start, &startTime);

    if( this->goalPose.get<0>() > RRTPlanner::maxXvalue ||  this->goalPose.get<0>() < RRTPlanner::minXvalue ||
        this->goalPose.get<1>() > RRTPlanner::maxYvalue || this->goalPose.get<1>() < RRTPlanner::minYvalue ){
        LOG_WARN( logger,"cel poza dopuszczalnymi wspolrzednymi x:["<<RRTPlanner::minXvalue <<";"<< RRTPlanner::maxXvalue<<"], y:["
        		<<RRTPlanner::minYvalue<<";"<< RRTPlanner::maxYvalue<<"]" );
        return RRTPlanner::BadTarget;
    }

	const double minDistance=Config::getInstance().getRRTMinDistance();
	GameStatePtr extendedGameState;
	bool collision=false;

	this->nearest=root;

	Pose startRobotPose=root->getRobotPos(robotId);

	Vector2D robotVelocity=(root->state)->getRobotVelocity(this->robotId);

	startRobotPose = Pose(startRobotPose.get<0>()+robotVelocity.x*deltaSimTime,
						startRobotPose.get<1>()+robotVelocity.y*deltaSimTime,
						startRobotPose.get<2>() );

	if( this->obsPredictionEnabled )
		evaluateEnemyPositions( (*root).state, deltaSimTime );

	this->initObstacles(root->getRobotPos(robotId) );

	//sprawdz czy aktualnie robot  nie jest w kolizji
	double safetyMarigin=0;
	bool checkAddObstacles=false;
	collision=isTargetInsideObstacle(startRobotPose,safetyMarigin,checkAddObstacles);

	if(collision){
		LOG_DEBUG(logger,this->robotName<<" robot is inside obstacle. collision");
		return RRTPlanner::RobotCollision;
	}

	// sprawdzic czy pkt docelowy nie jest w obrebie przeszkody
    safetyMarigin=0;
	collision=isTargetInsideObstacle(goalPose,safetyMarigin);
	if(collision){
		LOG_DEBUG(logger,"Achtung !!!"<<this->robotName<<" target is inside obstacle.");
		return RRTPlanner::TargetInsideObstacle;
	}

    double distanceToTarget;
    //sprawdz czy robot jest u celu
	if( (distanceToTarget=goalPose.distance( startRobotPose  ) ) <= minDistance ){
		LOG_TRACE(logger,"we arrive the target ");
		this->finish=true;
		return RRTPlanner::RobotReachGoalPose;
	}

	//sprawdz czy cel jest bezposrednio osiagalny
	checkAddObstacles = true;
	if(this->checkTargetAttainability(startRobotPose,goalPose,checkAddObstacles)){
		//std::cout<<"root->state"<<(*(root->state))<<std::endl;
        double ms = measureTime(stop, &startTime);
        LOG_DEBUG(logger,"RRT, robot goes directly to goal. RRT time "<<ms<<" [ms]" );
        GameStatePtr gameState( new GameState( *this->root->getGameState() ) );
        gameState->updateRobotData(this->robotName,this->goalPose );
        RRTNodePtr node(new RRTNode(gameState,this->robotId));
        node->setFinal();
        this->root->addNode( node );
        this->goDirectToTarget=true;
		return RRTPlanner::Success;
	}

	//pozycja robota w kolejnym kroku algorytmu
    Pose nextRobotPose=nearest->getRobotPos(this->robotId);

    //zasieg robota w 1 kroku algorytmu
    //double robotReach=( (robotVelocity.x+robotVelocity.y) / 2 )*simTime;
	//robotReach = std::max<double>( fabs( robotReach ), Config::getInstance().getRRTRobotReach() );
    double robotReach = Config::getInstance().getRRTRobotReach();
    double minRobotReach = (distanceToTarget/this->maxNodeNumber)*2;

    LOG_TRACE( logger,"robotReach "<<robotReach<<" minRobotReach"<<minRobotReach );

	//odleglosc do najblizszej przeszkody
	double toNearestObstacleDist=0;
	//tymczasowy cel w koljenym kroku algorytmu
	Pose temporaryTarget;
	//numer ostatnio dodanego wezla
	unsigned int nodeNr=0;


	//Pose tmpGoalPose = goalPose;
	//bool isGoalPose;
	this->blockedGoalPose = false;
	/*Główna pętla algorytmu RRT
	 *
	 *
	 *
	 */
	std::list<Pose> tmpWayPoints=this->wayPoints;
	while( (goalPose.distance( nextRobotPose ) > minDistance ) && nodeNr< RRTPlanner::maxNodeNumber ) {
		//wybieram tymczasowy pkt docelowy w zaleznosci od odleglosci robota do najblizszej przeszkody

		if( measureTime(stop, &startTime) > 200 ){
			LOG_FATAL(logger," FATAL RRT take over than 200 ms");
			break;
		}

		/*jesli poszerzenie najblizszego wezla w kierunku goalPose jest niemozliwe (przeszkoda)
		 * to chose target nie bieze goalPose nie jest brane pod uwage
		 *
		 */
		int  targetType;

		temporaryTarget = this->choseTarget(goalPose, &targetType , &tmpWayPoints);
		//temporaryTarget=this->choseTarget( goalPose , &isGoalPose);

		//wyszukuje pkt w drzewie najblizej wybranego celu tymczasowego
		nearest=findNearestState(temporaryTarget);

		if(nearest->getRobotPos(this->robotId).distance( temporaryTarget ) > minDistance){

			if( targetType==GOALPOSE ){
				//rozszerzam do celu
				extendedGameState=this->extendState( nearest->state,temporaryTarget,minRobotReach);
				if(extendedGameState.get()==NULL){
					this->blockedGoalPose = true;
				}
			}
			else if( targetType==WAYPOINT ){
				extendedGameState=this->extendState( nearest->state,temporaryTarget,robotReach);
				if(extendedGameState.get()==NULL){
					size_t s=tmpWayPoints.size();
					tmpWayPoints.remove(temporaryTarget);
					size_t ss=tmpWayPoints.size();
					assert(ss<s);
					//LOG_TRACE( logger, "remove waypoint size before "<<s<<" size after "<<ss );
				}
			}
			else{
				//rozszerzam do losowego celu
				extendedGameState=this->extendState( nearest->state,temporaryTarget,robotReach);
			}

			if( extendedGameState.get()!=NULL ){
				tmpWayPoints=this->wayPoints;
				this->blockedGoalPose = false;
				nodeNr++;
				RRTNodePtr node(new RRTNode(extendedGameState,this->robotId));
				node->setTargetPose(temporaryTarget);
				LOG_TRACE(logger,"addNode "<< (*node));
				nearest->addNode(node);
				nextRobotPose=node->getMyRobotPos();
				//toNearestObstacleDist=this->distanceToNearestObstacle( extendedGameState,nextRobotPose);
				toNearestObstacleDist=this->distanceToNearestObstacle(nextRobotPose);
			}
		}
		else if(targetType==GOALPOSE){
			this->blockedGoalPose = true;
		}
		else if(targetType==WAYPOINT){
			size_t s=tmpWayPoints.size();
			tmpWayPoints.remove(temporaryTarget);
			size_t ss=tmpWayPoints.size();
			assert(ss<s);
			//tmpWayPoints.remove(temporaryTarget);
			//LOG_TRACE( logger, "remove waypoint" );
		}
	}

	LOG_TRACE(logger,"end rrt with result "<<nearest->getRobotPos(this->robotId) <<" node amount "<<nodeNr);
	/*
	if(resultNode.get()==NULL){
		if(!this->root->children.empty()){
			resultNode = (*( std::min_element(this->root->children.begin(),this->root->children.end(),
				boost::bind(std::less<double>(),
						boost::bind(&RRTNode::shortestDistance,_1),
						boost::bind(&RRTNode::shortestDistance,_2) ) ) ) ) ;
		}
	}*/

	if(goalPose.distance( nextRobotPose ) > minDistance){
		nearest=findNearestState(goalPose);
		//this->path->clear();
	}
	else{
		this->foundNewPlan = true;
		this->path->clear();
	}

	//znajdz wezel najblizej celu ale tez bezposrednio osiagalny
	this->resultNode=findNearestAttainableState(goalPose);

	if(resultNode.get()!=NULL)
		resultNode->setFinal();
	else{
		resultNode=this->root;
		resultNode->setFinal();
	}


	double ms = measureTime(stop, &startTime);
    LOG_DEBUG( logger,"RRT time "<<ms<<" [ms]"<<" node amount "<<nodeNr<<" path size "<<this->path->size()
    		<<" foundNewPath"<<this->foundNewPlan  );

	return RRTPlanner::Success;

}

void RRTPlanner::initObstacles(const Pose& robotPose ){
	LOG_TRACE(logger,"initObstacles");
	this->obstacles.clear();
	const std::vector<std::string> blueTeam=Config::getInstance().getBlueTeam();

	BOOST_FOREACH(std::string modelName,blueTeam){
		if(modelName.compare(this->robotName)!=0){
			this->obstacles.push_back( root->getRobotPos( Robot::getRobotID( modelName) ) );
		}
	}

	const std::vector<std::string> redTeam=Config::getInstance().getRedTeam();
	BOOST_FOREACH(std::string modelName,redTeam){
		if(modelName.compare(this->robotName)!=0){
			this->obstacles.push_back( root->getRobotPos( Robot::getRobotID(modelName) ) );
		}
	}

	std::sort(this->obstacles.begin(),this->obstacles.end(),
			boost::bind<bool>(std::less<double> (),
				boost::bind(&Pose::distance,_1,robotPose),
				boost::bind(&Pose::distance,_2,robotPose)));

}


GameStatePtr RRTPlanner::getNextState(){
	LOG_TRACE(logger,"getNextState");

	if(this->finish){
        return GameStatePtr();
    }

    if( this->goDirectToTarget ){
        GameStatePtr gameState(new GameState( *(this->root->state) ));
        gameState->updateRobotData(this->robotName,this->goalPose);
        //std::cout<<"jestem w"<<this->root->state->getRobotPos(this->robotName)<<std::endl;
        //std::cout<<"jade do celu "<<gameState->getRobotPos(this->robotName)<<std::endl;
        return gameState;
    }

	if(this->root->children.empty()){
		return this->root->getGameState();
		//assert(false);
        //return GameStatePtr();
	}

	if(this->resultNode.get()!=NULL)
		return this->resultNode->state;

	//return (*( std::max_element(this->root->children.begin(),this->root->children.end(),
	//				boost::bind(std::less<double>(),
	//						boost::bind(&RRTNode::shortestDistance,_1),
	//						boost::bind(&RRTNode::shortestDistance,_2) ) ) ) )->state;
	return GameStatePtr();

}

Pose RRTPlanner::choseTarget( Pose goalPose, int * targetType, std::list<Pose>* wayPoints_ ){
	Pose result;
	*targetType=RANDOMPOINT;

	double t=uniformGen_01();
	//jesli w poprzednim kroku znaleziono sciezke do celu
	if( this->path->size()!=0 ){
		if( t<0.1 ){
			if(!blockedGoalPose){
				result=goalPose;
				*targetType=GOALPOSE;
			}
			else{
				result=getRandomPose();
				result.get<2>()=0;
			}
		}
		//way point
		else if (t<0.7 && wayPoints_->size() > 0 ){
			*targetType=WAYPOINT;
			//double p=1.0/this->path->size();
			double p=1.0/wayPoints_->size();
			double g=uniformGen_01();
			//wybieram jako docelowy jeden z wezlow ze sciezki znalezionej w poprzednim uruchomieniu RRT
			//std::list<Pose>::iterator ii=this->path->begin();
			//for(int i=1;ii!=this->path->end();ii++,i++){
			std::list<Pose>::iterator ii=wayPoints_->begin();
			for(int i=1;ii!=wayPoints_->end();ii++,i++){
				if(g<p*i){
					result=*ii;
					break;
				}
			}
		}
		else{
			result=getRandomPose();
		}
	}
	//jesli nie znaleziono wczesniej sciezki do celu
	else{
		if( t < this->toTargetLikelihood && (!blockedGoalPose) ){

			result=goalPose;
			*targetType=GOALPOSE;
			//*isGoalPose = true;
		}
		else
			result=getRandomPose();
	}

	assert( result.get<0>() > 0);
	assert( result.get<1>() > 0);

	LOG_TRACE(logger,"choseTarget. new target="<<result);

	return result;
}

RRTNodePtr RRTPlanner::findNearestState(const Pose & targetPose){

	//korzen tez bierze udział w poszukiwaniu najblizszego punktu
	RRTNodePtr result=this->root;
	double distance;
	this->shortestDist=0;

	Pose robotPose=root->getMyRobotPos();
	this->root->shortestDistance=robotPose.distance(targetPose);

	//znajdz potomka najblizej celu
	BOOST_FOREACH(RRTNodePtr node,this->root->children){
		result=findNearest(targetPose,node);
		if(result.get()!=NULL){
			robotPose=result->getMyRobotPos();
			this->root->shortestDistance=robotPose.distance(targetPose);
			this->shortestDist=distance=robotPose.distance(targetPose);

		}
	}
	//sprawdz czy przypadkiem korzen nie jest blizej celu
	//ale unikaj nadmiernego rozgalezienia
	if( this->root->children.size() < maxRootChildren ){
		if(this->root->getMyRobotPos().distance(targetPose) < distance ){
			result=this->root;
			this->shortestDist=this->root->shortestDistance=this->root->getMyRobotPos().distance(targetPose);
		}
	}
	LOG_TRACE(logger,"findNearestState "<<result->getMyRobotPos()<< " to target "<<targetPose);
	return result;
}

RRTNodePtr RRTPlanner::findNearest(const Pose & targetPose,RRTNodePtr & currNode){
	//LOG_TRACE(logger,"findNearest");

	RRTNodePtr result;
	Pose robotPose=currNode->getMyRobotPos();
	currNode->shortestDistance=robotPose.distance(targetPose);
	result=currNode;

	//wezel potomny najblizej celu
	RRTNodePtr nearest;

	BOOST_FOREACH(RRTNodePtr node,currNode->children){
		nearest=findNearest(targetPose,node);
		if( nearest->shortestDistance < currNode->shortestDistance ){
			currNode->shortestDistance=nearest->shortestDistance;
			result=nearest;
		}
	}
	return result;
}

RRTNodePtr RRTPlanner::findNearestAttainableState(const Pose & targetPose){
	LOG_TRACE(logger,"findNearestAttainableState");
	//korzen tez bierze udział w poszukiwaniu najblizszego punktu
	RRTNodePtr tmpResult;
	RRTNodePtr result;

	Pose startRobotPose=root->getMyRobotPos();

	//znajdz bezposrednio osiagalnego potomka najblizej celu
	BOOST_FOREACH(RRTNodePtr node,this->root->children){
		tmpResult=findNearestAttainableState(targetPose,node);
		//sprawdz potomkow korzenia pod wzgledem osiagalnosci i odleglosci
		if(tmpResult.get()!=NULL){
		    Pose robotPose=tmpResult->getMyRobotPos();
			if( result.get()==NULL || tmpResult->shortestDistance < result->shortestDistance ){
				if(checkTargetAttainability( startRobotPose, robotPose)==true){
					result=tmpResult;;
				}
			}

		}
	}

	if(result.get()==NULL)
		return this->root;

	return result;
}
RRTNodePtr RRTPlanner::findNearestAttainableState(const Pose & targetPose,RRTNodePtr currNode){
	LOG_TRACE(logger,"findNearestAttainableState");
	RRTNodePtr result;
	//wezel potomny najblizej celu
	RRTNodePtr nearest;

	Pose startRobotPose=root->getMyRobotPos();
	Pose robotPose;

	BOOST_FOREACH(RRTNodePtr node,currNode->children){
		nearest=findNearestAttainableState(targetPose,node);
		if(nearest.get()!=NULL){
			robotPose=nearest->getMyRobotPos();
			if(result.get()==NULL || nearest->shortestDistance < result->shortestDistance ){
				if(checkTargetAttainability( startRobotPose, robotPose)==true){
					result=nearest;
				}
			}
		}
	}
	robotPose=currNode->getMyRobotPos();
	if(this->foundNewPlan)
		if(currNode->shortestDistance<=this->shortestDist)
			path->push_back(robotPose);

	if(result.get()==NULL || currNode->shortestDistance < result->shortestDistance){
		if(checkTargetAttainability( startRobotPose, robotPose)==true){
			result=currNode;
		}
	}

	return result;
}

Pose RRTPlanner::getRandomPose(){

	//generator wspolrzednej X
	static boost::mt19937 rngX(static_cast<unsigned> (time(NULL)));
	//generator wspolrzednej Y
	static boost::mt19937 rngY(static_cast<unsigned> (2*time(NULL)));

	//interesujace sa jedynie pozycje rozniace sie co najwyzej o 0.01 [m]
	static boost::uniform_int<int> uni_distX(this->minXvalue*10,this->maxXvalue*10);

	static boost::variate_generator<boost::mt19937, boost::uniform_int<int> >
		genX(rngX, uni_distX);

	//interesujace sa jedynie pozycje rozniace sie co najwyzej o 0.01 [m]
	static boost::uniform_int<int> uni_distY(this->minYvalue*10,this->maxYvalue*10);

	static boost::variate_generator<boost::mt19937, boost::uniform_int<int> >
		genY(rngY, uni_distY);

	double x=genX();
	double y=genY();

	Pose randomPose(x/10.0,y/10.0,0);

	return randomPose;
}

Pose RRTPlanner::getRandomPose(Pose currentPose){
	LOG_TRACE(logger,"getRandomPose");
	Pose robotPose=currentPose;
	//Pose robotPose=currentState.get()->getRobotPos(this->robotName);
	static boost::mt19937 rng(static_cast<unsigned> (time(NULL)));

	//generator wspolrzednej X
	static boost::normal_distribution<double> distX(robotPose.get<0>(),RRTPlanner::randomStateReach);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> >  genX(rng, distX);

	//generator wspolrzednej Y
	static boost::normal_distribution<double> distY(robotPose.get<1>(),RRTPlanner::randomStateReach);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> >  genY(rng, distY);

	return Pose(genX(),genY(),robotPose.get<2>());
}

Pose RRTPlanner::getRandomPose(const Pose currentPose,Vector2D currentVel, double deltaVel){
	LOG_TRACE(logger,"getRandomPose");
	std::ostringstream ois;
	static double deltaT=0.1;//0.1  sekund
	Vector2D v1=Vector2D (currentVel.x+deltaVel,currentVel.y-deltaVel);
	Vector2D v2=Vector2D (currentVel.x-deltaVel,currentVel.y+deltaVel);

	ois<<"current Pose "<<currentPose<<" currentVel"<<currentVel<<" v1 "<<v1<<" v2 "<<v2<<std::endl;
	LOG_DEBUG(logger,ois);
	static boost::mt19937 rng(static_cast<unsigned> (time(NULL)));

	//generator wspolrzednej X
/*	double a =100;
	double b =200;
	double c =150;

//	std::cout<<"dla distX a="<<a<<" b="<<b<< "c="<<(currentPose.get<0>()+ velocity.x )*100 <<std::endl;
//	boost::triangle_distribution<double> distX(a, c, b );
//	boost::variate_generator<boost::mt19937&, boost::triangle_distribution<double> >  genX(rng, distX);
*/
//	std::cout<<"velocity "<<velocity<<std::endl;
	//generator wspolrzednej X

	//double a =std::min<double>(currentPose.get<0>()+v1.x, currentPose.get<0>()+v2.x) ;
	//double b =std::max<double>(currentPose.get<0>()+v1.x, currentPose.get<0>()+v2.x) ;
	double a,b;
	if(fabs(currentVel.x)<0.01){
		a = 0;
		b = 5.4;//wymiar boiska
	}
	else if(currentVel.x>0){
		a = currentPose.get<0>();
		b = 5.4;//wymiar boiska
	}
	else{
		a = 0;
		b = currentPose.get<0>();
	}
	double c=std::max<double>(0,currentPose.get<0>()+currentVel.x*deltaT);
	ois.str("");
	ois<<"dla distX a="<<a<<" b="<<b<< "c="<<c <<std::endl;
	LOG_DEBUG(logger,ois);
	boost::triangle_distribution<double> distX(a, c, b );
	boost::variate_generator<boost::mt19937&, boost::triangle_distribution<double> >  genX(rng, distX);

	//generator wspolrzednej Y
	if(fabs(currentVel.y)<0.01){
		a = 0;
		b = 7.4;//wymiar boiska
	}
	else if(currentVel.y>0){
		a = currentPose.get<1>();
		b = 7.4;//wymiar boiska
	}
	else{
		a = 0;
		b = currentPose.get<1>();
	}
	c=std::max<double>(0,currentPose.get<1>()+currentVel.y*deltaT);
	ois.str("");
	ois<<"dla distY a="<<a<<" b="<<b<< "c="<<c <<std::endl;
	LOG_DEBUG(logger,ois);
	boost::triangle_distribution<double> distY(a, c , b);
	boost::variate_generator<boost::mt19937&, boost::triangle_distribution<double> >  genY(rng, distY);

	double x=genX();
	double y=genY();

	Pose randomPose(x,y,0.0);
	return randomPose;
}

GameStatePtr RRTPlanner::extendState( const GameStatePtr & currState, const Pose& targetPose, const double robotReach_ ){
	Pose currPose=currState->getRobotPos(this->robotId);

	LOG_TRACE(logger," extendState " << " currPose "<<currPose<<" targetPos "<<targetPose );
	assert(currState.get()!=NULL);
	double robotReach=robotReach_;
	bool checkAdditionalObstacles = true;
	//sprawdzenie czy targetPose nie lezy w obrebie przeszkody
	if( isTargetInsideObstacle( targetPose, RRTPlanner::SAFETY_MARGIN , checkAdditionalObstacles ) ){
		//LOG_TRACE(logger,"targetPose inside obstacle");
		return GameStatePtr();
	}

	GameStatePtr result( new GameState(*currState));

	//wyznacz rownanie prostej laczacej stan biezacy z docelowym
	double x=0,y=0;
	//jesli cel jest blizej niz RRT::RobotReach
	if(fabs( targetPose.distance(currPose) ) < robotReach){
		 robotReach=fabs(targetPose.distance(currPose) );
	}
	//jesli jest to prosta typu x=A
	if( fabs(currPose.get<0>()-targetPose.get<0>() ) < 0.001 ){
		//result->updateRobotData(this->robotName,
		//	Pose(currPose.get<0>(),currPose.get<1>()+robotReach,currPose.get<2>()),
		//			currState->getRobotVelocity(this->robotName)
		//	);
		x=currPose.get<0>();
		y=currPose.get<1>()+robotReach;
	}
	else{

		double xA=currPose.get<0>();
		double yA=currPose.get<1>();

		double xB=targetPose.get<0>();
		double yB=targetPose.get<1>();

		//wsp kierunkowy prostej
		double a=(yB-yA)/(xB-xA);

		//przesuniecie
		double b=yA - xA*a;

		double delta = /*4*/( pow( a*(b - yA) - xA,2 ) - (1+pow(a,2))*(pow(xA,2)+ pow((b-yA),2) -
			pow(robotReach,2) ) );

		assert(delta>0);
		double x1=( xA-a*(b-yA)-sqrt(delta) )/(1+pow(a,2));
		double x2=( xA-a*(b-yA)+sqrt(delta) )/(1+pow(a,2));

		if(targetPose.get<0>() > currPose.get<0>()){
			x=std::max(x1,x2);
			y=a*x+b;
		}
		else{
			x=std::min(x1,x2);
			y=a*x+b;
		}
	}

	result->updateRobotData(this->robotName,
				Pose(x,y,goalPose.get<2>()) );

	Pose resultPose=result->getRobotPos(this->robotId);
	double test=pow( resultPose.get<0>()-currPose.get<0>(),2 )+pow( resultPose.get<1>()-currPose.get<1>(),2 );
	//test poprawnosci rozwiazania
	assert( fabs( pow(robotReach,2 ) - test ) < 0.001 );
	/*
	double res_test=fabs( pow(robotReach, 2 ) - test);
	if( !( res_test < 0.001) ){
		LOG_FATAL(
				this->logger,
				"assert( fabs( pow(robotReach,2 ) - test ) < 0.001 ) FAILD. test"<<test
				<<" res_test "<<res_test <<" robotReach "<<robotReach
				<<" currPose "<<currPose<<" targetPos "<<targetPose<<" resultPose "<<resultPose );
		exit(0);
	}*/

	//czy ten punkt nie lezy w obrebie przeszkody
	if(isTargetInsideObstacle(  resultPose , RRTPlanner::SAFETY_MARGIN, checkAdditionalObstacles ) ){
		return GameStatePtr();
	}

	//double angle=currState->getRobotVelocity(this->robotName).angleTo(Vector2D(x,y));

	//result->updateRobotData(this->robotName,
	//		Pose(x,y,currPose.get<2>()),
	//				currState->getRobotVelocity(this->robotName).rotate(angle) );

    result->updateRobotData(this->robotName,
			Pose(x,y,currPose.get<2>()),
					currState->getRobotVelocity(this->robotId) );

	return result;
}
void RRTPlanner::evaluateEnemyPositions(const GameStatePtr & currState,const double deltaSimTime){
	LOG_TRACE(logger,"evaluateEnemyPositions");
	const std::vector<std::string> blueTeam=Config::getInstance().getBlueTeam();

	BOOST_FOREACH(std::string modelName,blueTeam){
		if(modelName.compare(this->robotName)!=0){
			Vector2D v=(*currState).getRobotVelocity( Robot::getRobotID(modelName) );
			Pose rPose=(*currState).getRobotPos( Robot::getRobotID(modelName) );
			Pose newPose(rPose.get<0>()+v.x*deltaSimTime,rPose.get<1>()+v.y*deltaSimTime,0);
			this->predictedObstaclesPos.push_back(newPose);
			//std::cout<<"add obstacle blue "<<nr++<<" robot velocity "<<v<<std::endl;
			//(*currState).updateRobotData(modelName,newPose,v,0);
		}
	}

	const std::vector<std::string> redTeam=Config::getInstance().getRedTeam();
	BOOST_FOREACH(std::string modelName,redTeam){
		if(modelName.compare(this->robotName)!=0){
			Vector2D v=(*currState).getRobotVelocity( Robot::getRobotID(modelName) );
			Pose rPose=(*currState).getRobotPos( Robot::getRobotID(modelName) );
			Pose newPose(rPose.get<0>()+v.x*deltaSimTime,rPose.get<1>()+v.y*deltaSimTime,0);
			this->predictedObstaclesPos.push_back(newPose);
			//std::cout<<"add obstacle red "<<nr++<<" robot velocity "<<v<<std::endl;
			//(*currState).updateRobotData(modelName,newPose,v,0);
		}
	}
}

bool RRTPlanner::checkTargetAttainability(const Pose &currPose,const Pose &targetPose,bool checkAddObstacles){
	LOG_TRACE(logger,"checkTargetAttainability");

	double robotRadius=Config::getInstance().getRRTRobotRadius()+RRTPlanner::SAFETY_MARGIN;

	Vector2D tar=( targetPose.getPosition() - currPose.getPosition()  );

	//transformacja do lokalnego ukl wspolrzednych
	//os OY na wprost do celu
	double teta_cel=atan2( (tar.y) , (tar.x));
	RotationMatrix rm( convertAnglePI(teta_cel - M_PI/2) );

	double A,B,C;
	//jesli jest to prosta typu x=A
	if( fabs(currPose.get<0>()-targetPose.get<0>() ) < 0.001 ){
		A=1;
		B=0;
		C=-currPose.get<0>();
	}
	//rownanie prostej laczacej robota i pkt docelowy
	else{
		A = ( currPose.get<1>()-targetPose.get<1>() )/(currPose.get<0>()-targetPose.get<0>());
		B = -1;
		C=A*(-targetPose.get<0>()) + targetPose.get<1>();
	}

#ifdef DEBUG
    Pose targetPose_r=targetPose.transform(currPose.getPosition(),rm);
	LOG_DEBUG(logger,"biezaca pozycja globalnie "<<currPose<<"pozycja celu globalnie "<<targetPose<<" oraz w ukl zw z robotem "<<targetPose_r<<std::endl;
#endif

	double d;
	//pozycja przeszkody w ukladzie zw z robotem
	Pose obstaclePose_r;
	//sprawdz czy odcinek laczacy pkt biezacy i docelowy nie przechodzi przez przeszkode
	BOOST_FOREACH(Pose obstaclePose,this->obstacles){
#ifdef DEBUG
		//std::cout<<"check collision with "<<obstaclePose<<std::endl;
#endif
		//jesli odleglosc  srodka przeszkody od prostej laczacej robota i cel jest <= R przeszkody to mamy kolizje
		d=fabs( A*obstaclePose.get<0>() + B*obstaclePose.get<1>() +C )/sqrt(pow(A,2)+pow(B,2));

		obstaclePose_r=obstaclePose.transform(currPose.getPosition(),rm);

#ifdef DEBUG
		std::cout<<"obstacle position in robot coordinates "<<obstaclePose_r<<std::endl;
#endif
		if(  ( d<=robotRadius ) &&
				//odleglosc robota do przeszkody jest mniejsza niz robota od celu
				( fabs(currPose.distance(obstaclePose) ) <= fabs(currPose.distance(targetPose) ) ) &&
				//przeszkoda jest przed robotem
				obstaclePose_r.get<1>()>0){
#ifdef DEBUG
			//std::cout<<"collision with "<<obstaclePose<<std::endl;
#endif
			return false;
		}
	}

	if(checkAddObstacles){
		//sprawdz czy odcinek laczacy pkt biezacy i docelowy nie przechodzi przez przeszkode
		BOOST_FOREACH(Pose obstaclePose,this->predictedObstaclesPos){
	#ifdef DEBUG
			//std::cout<<"check collision with "<<obstaclePose<<std::endl;
	#endif
			//jesli odleglosc  srodka przeszkody od prostej laczacej robota i cel jest <= R przeszkody to mamy kolizje
			d=fabs( A*obstaclePose.get<0>() + B*obstaclePose.get<1>() +C )/sqrt(pow(A,2)+pow(B,2));

			obstaclePose_r=obstaclePose.transform(currPose.getPosition(),rm);

	#ifdef DEBUG
			std::cout<<"obstacle position in robot coordinates "<<obstaclePose_r<<std::endl;
	#endif
			if(  ( d<=robotRadius ) &&
					//odleglosc robota do przeszkody jest mniejsza niz robota od celu
					( fabs(currPose.distance(obstaclePose) ) <= fabs(currPose.distance(targetPose) ) ) &&
					//przeszkoda jest przed robotem
					obstaclePose_r.get<1>()>0){
	#ifdef DEBUG
				//std::cout<<"collision with "<<obstaclePose<<std::endl;
	#endif
				return false;
			}
		}
	}
	return true;
}
/**
 * @brief
 * @param [in] currPose biezaca pozycja robota w globalnym ukladzie wspolrzednych
 * @param [in] targetPose pozycja celu w globalnym ukladzie wsplorzednych
 */
bool RRTPlanner::isTargetInsideObstacle(const Pose &targetPose, double safetyMarigin,bool checkAddObstacles){

	const double robotRadius=Config::getInstance().getRRTRobotRadius();

	//sprawdz czy punkt docelowy nie lezy w obrebie przeszkody
	BOOST_FOREACH(Pose obstaclePose,this->obstacles){
		if(  pow(targetPose.get<0>()-obstaclePose.get<0>(),2) + pow(targetPose.get<1>()-obstaclePose.get<1>(),2) <=
						pow(robotRadius+safetyMarigin,2) ){
			LOG_TRACE(logger,"isTargetInsideObstacle true");
			return true;
		}
	}

	if(checkAddObstacles){
		//sprawdz czy punkt docelowy nie lezy w obrebie przewidywanego polozenia przeszkody
		BOOST_FOREACH(Pose obstaclePose,this->predictedObstaclesPos){
			if(  pow(targetPose.get<0>()-obstaclePose.get<0>(),2) + pow(targetPose.get<1>()-obstaclePose.get<1>(),2) <=
							pow(robotRadius+safetyMarigin,2) ){
				LOG_TRACE(logger,"isTargetInsideObstacle true addObs");
				return true;
			}
		}
	}
	LOG_TRACE(logger,"isTargetInsideObstacle false");
	return false;
}
/*
double RRTPlanner::distanceToNearestObstacle(const GameStatePtr & currState,const Pose &currPose){
	Pose obstaclePose;
	const std::vector<std::string> blueTeam=Config::getInstance().getBlueTeam();
	const std::vector<std::string> redTeam=Config::getInstance().getBlueTeam();

	std::ostringstream log;
	double distance=numeric_limits<double>::max();

	BOOST_FOREACH(std::string modelName,blueTeam){
		if(modelName.compare(this->robotName)!=0){
			obstaclePose=currState->getRobotPos(modelName);
			distance=std::min<double>(distance,( currPose.distance(obstaclePose)+Config::getInstance().getRRTRobotRadius() ));
		}
	}

	BOOST_FOREACH(std::string modelName,redTeam){
			if(modelName.compare(this->robotName)!=0){
				obstaclePose=currState->getRobotPos(modelName);
				distance=std::min<double>(distance,currPose.distance(obstaclePose)+Config::getInstance().getRRTRobotRadius() );
			}
		}
	return distance;
}
*/
double RRTPlanner::distanceToNearestObstacle(const Pose &currPose){
	LOG_TRACE(logger,"distanceToNearestObstacle");
	//Pose obstaclePose;
	//const std::vector<std::string> blueTeam=Config::getInstance().getBlueTeam();
	//const std::vector<std::string> redTeam=Config::getInstance().getBlueTeam();

	//std::ostringstream log;
	double distance=numeric_limits<double>::max();

    std::vector<Pose> obstacles;

	BOOST_FOREACH(Pose obstaclePose,obstacles){
        //obstaclePose=currState->getRobotPos(modelName);
		distance=std::min<double>( distance,( currPose.distance(obstaclePose) - Config::getInstance().getRRTRobotRadius() ));
		assert(distance > 0);
	}

	return distance;
}
int RRTPlanner::serializeTree(const char * fileName,int serializedTrees){
	LOG_TRACE(logger,"serializeTree");
	int status;
	xmlTextWriterPtr writer;
	xmlDocPtr doc;
	xmlNodePtr node;
	std::ostringstream ois;

	/* Create a new XML DOM tree, to which the XML document will be
	 * written */
	doc = xmlNewDoc(BAD_CAST XML_DEFAULT_VERSION);

	if (doc == NULL) {
		printf
			("testXmlwriterTree: Error creating the xml document tree\n");
		return -1;
	}


	/* Create a new XML node, to which the XML document will be
	 * appended */
	node = xmlNewDocNode(doc, NULL, BAD_CAST "RRTDebug", NULL);
	if (node == NULL) {
		printf("testXmlwriterTree: Error creating the xml node\n");
		return -1;
	}

	/* Make ELEMENT the root node of the tree */
	xmlDocSetRootElement(doc, node);

	/* Create a new XmlWriter for DOM tree, with no compression. */
	writer = xmlNewTextWriterTree(doc, node, 0);
	if (writer == NULL) {
		printf("testXmlwriterTree: Error creating the xml writer\n");
		return -1;
	}

	/* Start the document with the xml default for the version,
	 * encoding ISO 8859-1 and the default for the standalone
	 * declaration. */
	status = xmlTextWriterStartDocument(writer, NULL, MY_ENCODING, NULL);
	if (status < 0) {
		printf("testXmlwriterTree: Error at xmlTextWriterStartDocument\n");
		return status;
	}

	status = xmlTextWriterStartElement(writer, BAD_CAST "GameState");
	if (status < 0) {
		printf("testXmlwriterTree: Error at xmlTextWriterStartElement\n");
		return status;
	}
	ois.str("");
	ois<<serializedTrees;
	status = xmlTextWriterWriteAttribute(writer, BAD_CAST "nr",
									 BAD_CAST ois.str().c_str());
	if (status < 0) {
		printf("testXmlwriterTree: Error at xmlTextWriterWriteAttribute\n");
		return status;
	}

	ois.str("");
	ois<<this->simTime;
	status = xmlTextWriterWriteAttribute(writer, BAD_CAST "simTime",
									 BAD_CAST ois.str().c_str());
	if (status < 0) {
		printf("testXmlwriterTree: Error at xmlTextWriterWriteAttribute\n");
		return status;
	}


	ois.str("");
	ois<<this->robotName;
	status = xmlTextWriterWriteAttribute(writer, BAD_CAST "robotName",
									 BAD_CAST ois.str().c_str());
	if (status < 0) {
		printf("testXmlwriterTree: Error at xmlTextWriterWriteAttribute\n");
		return status;
	}

	root->serializeNodeToXml(writer);

	status = xmlTextWriterStartElement(writer, BAD_CAST "obstacles");
	if (status < 0) {
		printf("testXmlwriterTree: Error at xmlTextWriterStartElement\n");
		return status;
	}

	std::vector<Pose>::iterator ii=this->predictedObstaclesPos.begin();
	int obsNr=0;
	for(;ii!=this->predictedObstaclesPos.end();ii++){
		serializeRobotToXml(writer,"obstacle",*ii,obsNr++);
	}

	// Close the element named Obstacles
    status = xmlTextWriterEndElement(writer);
    if (status < 0) {
        printf("testXmlwriterTree: Error at xmlTextWriterEndElement in serializeTree\n");
        return status;
    }

	status = xmlTextWriterStartElement(writer, BAD_CAST "RRTTree");
	if (status < 0) {
		printf("testXmlwriterTree: Error at xmlTextWriterStartElement\n");
		return status;
	}

	//zapis drzewa w glab
	root->serializeRecursiveToXml(writer,this->robotName);

	// Close the element named RRTTree
    status = xmlTextWriterEndElement(writer);
    if (status < 0) {
        printf("testXmlwriterTree: Error at xmlTextWriterEndElement in serializeTree\n");
        return status;
    }


	// Close the element named GameState
    status = xmlTextWriterEndElement(writer);
    if (status < 0) {
        printf("testXmlwriterTree: Error at xmlTextWriterEndElement in serializeTree\n");
        return status;
    }

    status = xmlTextWriterEndDocument(writer);
    if (status < 0) {
        printf("testXmlwriterTree: Error at xmlTextWriterEndDocument\n");
        return status;
    }

    xmlFreeTextWriter(writer);

    if(serializedTrees==0){
    	std::string header("<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>");
    	ofstream myfile (fileName,ios::out | ios::trunc);
    	myfile<<header<<std::endl;
    	myfile.close();


    	int fd=open(fileName, O_RDWR);
    	if(fd<0){
    		std::cout<<"cant create file "<<fileName<<std::endl;
    		std::cout<<"error when opening file open ret"<<fd<<" "<<strerror(errno)<<std::endl;
    	}
    	//else
    	//	std::cout<<"open file corectly"<<std::endl;

    	//write(fd,header,strlen(header));

    	xmlSaveCtxtPtr ctxt= xmlSaveToFd(fd,MY_ENCODING, XML_SAVE_NO_EMPTY);
    	xmlSaveTree(ctxt,node);
    	xmlSaveFlush(ctxt);
    	close(fd);
    }
    else{
    	//</GameState>
    	//</RRTDebug>
    	//fseek(file, 0, SEEK_END);
    	int fd=open(fileName, O_RDWR, 0);
    	lseek(fd, -11, SEEK_END);
    	xmlSaveCtxtPtr ctxt= xmlSaveToFd(fd,MY_ENCODING, XML_SAVE_NO_EMPTY);
    	xmlSaveTree(ctxt,node->children);
    	xmlSaveFlush(ctxt);
    	close(fd);

     	ofstream myfile (fileName,ios::out | ios::app);
       	myfile<<"</RRTDebug>";
       	myfile.close();
    }

    xmlFreeDoc(doc);
    serializedTrees++;

    return 0;
}
/*
Vector2D RRTPlanner::getRobotSpeed(){

    //biezaca rotacja robota
    //double robotRotation=(*currGameState).getRobotPos( robot->getRobotName()).get<2>() ;

    double robotRotation=this->root->getMyRobotPos().get<2>() ;

    //macierz obrotu os OY na wprost robota
    RotationMatrix rmY(robotRotation);

   // Pose nextRobotPose=this->resultState->;
    //pozycja celu w ukladzie wsp zwiazanych z robotem
    Pose targetRelPosition=rmY.Inverse()*(nextRobotPose.getPosition() - this->root->getMyRobotPos().getPosition());
    #ifdef DEBUG
        std::cout<<"targetRelPosition "<<targetRelPosition<<std::endl;
    #endif
    //pobiez biezaca predkosc robota
    currRobotVel=(*currGameState).getRobotVelocity( robot->getRobotName() );
    #ifdef DEBUG
        std::cout<<"currRobotVel "<<currRobotVel<<std::endl;
    #endif
    //oblicz nowe sterowanie
    newRobotVel=calculateVelocity( currRobotVel, Pose(targetRelPosition.x,targetRelPosition.y,0));
*/
//}

RRTPlanner::~RRTPlanner() {
	LOG_TRACE(logger,"~RRTPlanner");
	ofstream myfile;
	myfile.open ("rrtTree.xml", ios::out | ios::app);
	myfile<<std::endl;
	myfile.close();
}


std::ostream& operator<<(std::ostream& out, enum RRTPlanner::ErrorCode errcode ){
    switch(errcode){
        case RRTPlanner::Success:
        out<<"Success";
        break;

        case RRTPlanner::RobotReachGoalPose:
        out<<"RobotReachGoalPose";
        break;

        case RRTPlanner::TargetInsideObstacle:
        out<<"TargetInsideObstacle";
        break;

        case RRTPlanner::RobotCollision:
        out<<"RobotCollision";
        break;

        case RRTPlanner::BadTarget:
        out<<"Badtarget";
        break;
        default:
            break;
    }

    return out;

}
