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

RRTPlanner::RRTPlanner(const double goalProb,const std::string robotName,bool withObsPrediction,
			const GameStatePtr currState,const Pose goalPose_,std::list<Pose> * path):
                root(new RRTNode(currState,robotName)),goalPose(goalPose_),
                toTargetLikelihood(goalProb),obsPredictionEnabled(withObsPrediction){

    goDirectToTarget=false;
    finish=false;

	//clock_t startTime=clock();
	//clock_t currTime;
	//long long int clocks_per_msec=CLOCKS_PER_SEC/1000;
	//std::cout<<"CLOCKS_PER_SEC"<<CLOCKS_PER_SEC<<"clocks_per_msec "<<clocks_per_msec<<std::endl;
	//std::cout<<"startTime"<<startTime<<std::endl;
	this->path=path;
	std::ostringstream log;

	#ifdef DEBUG
		log<<"starting rrt goal Pose "<<goalPose<<" and currPose "<<currState->getRobotPos(robotName);
		Logger::getInstance().LogToFile(DBG,log);
	#endif

	this->robotName=robotName;

	//this->initObstacles(root->getRobotPos(this->robotName) );
}

bool RRTPlanner::run(const GameStatePtr currState,double deltaSimTime){
	std::ostringstream log;
	const double minDistance=Config::getInstance().getRRTMinDistance();
	GameStatePtr extend;
	bool noCollision=true;

	this->nearest=root;

	Pose startRobotPose=root->getRobotPos(robotName);

	//sprawdz czy aktualnie robot  nie jest w kolizji
	double safetyMarigin=0;
	noCollision=isTargetInsideObstacle(startRobotPose,safetyMarigin);

	Vector2D v=(*currState).getRobotVelocity(this->robotName);
	startRobotPose = Pose(startRobotPose.get<0>()+v.x*deltaSimTime,startRobotPose.get<1>()+v.y*deltaSimTime,0);

/*
	//sprawdz czy robot nie lezy w obrebie przeszkody
	BOOST_FOREACH(Pose obstaclePose,this->obstacles){
		if(  pow( startRobotPose.get<0>()-obstaclePose.get<0>(),2) + pow(startRobotPose.get<1>()-obstaclePose.get<1>(),2) <=
							pow(Config::getInstance().getRRTRobotRadius(),2) ){
			noCollision=false;
			break;
		}
	}*/

	//TODO: ew sprawdzic czy pkt docelowy nie jest w obrebie przeszkody
	if(this->obsPredictionEnabled)
		evaluateEnemyPositions( (*root).state,PREDICTION_TIME);

	this->initObstacles(root->getRobotPos(robotName) );

	//pozycja robota w kolejnym kroku algorytmu
    Pose nextRobotPose=nearest->getRobotPos(this->robotName);
    double dist;
    //sprawdz czy robot nie jest u celu
	if( (dist=goalPose.distance( nextRobotPose ) ) <= minDistance ){
		std::cout<<"we arrive the target"<<std::endl;
		this->finish=true;
	}

	std::cout<<"odleglosc do celu"<<dist<<std::endl;


	//sprawdz czy cel nie jest bezposrednio osiagalny
	bool checkAddObstacles = true;
	if(this->checkTargetAttainability(startRobotPose,goalPose,checkAddObstacles)){
		//TODO: ustawic odpowiedni wskaznik
		std::cout<<"cel jest bezposrednio osiagalny"<<std::endl;
		this->goDirectToTarget=true;
		return true;
	}


	if(noCollision){
		//zasieg robota w 1 kroku algorytmu
		double robotReach=Config::getInstance().getRRTRobotReach();
		//biezaca predkosc robota
		Vector2D robotVelocity=currState->getRobotVelocity(this->robotName);
		//odleglosc do najblizszej przeszkody
		double toNearestObstacleDist=0;
		//tymczasowy cel w koljenym kroku algorytmu
		Pose temporaryTarget;
		//ograniczenie na maksymalna liczbe wezłów w drzewie
		const unsigned int maxNodeNumber=400;
		//numer ostatnio dodanego wezla
		int nodeNr=0;


		while( (goalPose.distance( nextRobotPose ) > minDistance ) && nodeNr<maxNodeNumber) {

			//wybieram tymczasowy pkt docelowy w zaleznosci od odleglosci robota do najblizszej przeszkody
			temporaryTarget=this->choseTarget(goalPose,robotVelocity,nextRobotPose,toNearestObstacleDist);
			//wyszukuje pkt w drzewie najblizej wybranego celu
			nearest=findNearestState(temporaryTarget);
			if(nearest->getRobotPos(this->robotName).distance( temporaryTarget ) > minDistance){
				//rozszerzam do celu
				extend=this->extendState( nearest->state,temporaryTarget,robotReach);

				if(extend.get()!=NULL){
					nodeNr++;
					RRTNodePtr node(new RRTNode(extend,this->robotName));
					node->setTargetPose(temporaryTarget);
					nearest->addNode(node);
					nextRobotPose=node->getMyRobotPos();
					toNearestObstacleDist=this->distanceToNearestObstacle(extend,nextRobotPose);
				}
				else{
					Logger::getInstance().LogToFile(DBG,"extented node is in the obstacle");
				}
			}
/*
			currTime=clock();
			//std::cout<<"currTime"<<currTime<<std::endl;
			if( currTime > (startTime+10*clocks_per_msec) ){
				//std::cout<<"diffTime"<<(startTime-currTime)<<std::endl;
				break;
			}
			*/
		}

		#ifdef DEBUG
			log<<"end rrt result "<<nearest->getRobotPos(this->robotName)<<std::endl;
			Logger::getInstance().LogToFile(DBG,log);
		#endif

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
			this->path->clear();
		}

		this->resultNode=findNearestAttainableState(goalPose);

		if(resultNode.get()!=NULL)
			resultNode->setFinal();
		else{
			resultNode=this->root;
			resultNode->setFinal();
			std::cout<<this->robotName<<" result is NULL"<<std::endl;
		}
		return true;
	}
	else{
		std::cout<<this->robotName<<" collision"<<std::endl;
		return false;
	}

}
void RRTPlanner::initObstacles(const Pose& robotPose ){
	this->obstacles.clear();
	const std::vector<std::string> blueTeam=Config::getInstance().getBlueTeam();

	BOOST_FOREACH(std::string modelName,blueTeam){
		if(modelName.compare(this->robotName)!=0){
			this->obstacles.push_back(root->getRobotPos(modelName));
			//std::cout<<modelName<<std::endl;
		}
	}

	const std::vector<std::string> redTeam=Config::getInstance().getRedTeam();
	BOOST_FOREACH(std::string modelName,redTeam){
		if(modelName.compare(this->robotName)!=0){
			this->obstacles.push_back(root->getRobotPos(modelName));
			//std::cout<<modelName<<std::endl;
		}
	}

	std::sort(this->obstacles.begin(),this->obstacles.end(),
			boost::bind<bool>(std::less<double> (),
				boost::bind(&Pose::distance,_1,robotPose),
				boost::bind(&Pose::distance,_2,robotPose)));

}


GameStatePtr RRTPlanner::getNextState(){

    if(this->finish){
        std::cout<<"dojechalem do celu"<<std::endl;
        return GameStatePtr();
    }

    if(this->goDirectToTarget){
        GameStatePtr gameState(this->root->state);
        gameState->updateRobotData(this->robotName,this->goalPose);
        std::cout<<"jestem w"<<this->root->state->getRobotPos(this->robotName)<<std::endl;
        std::cout<<"jade do celu "<<gameState->getRobotPos(this->robotName)<<std::endl;
        return gameState;
    }
	if(this->root->children.empty()){
        return GameStatePtr();
	}

	if(this->resultNode.get()!=NULL)
		return this->resultNode->state;

	//return (*( std::max_element(this->root->children.begin(),this->root->children.end(),
	//				boost::bind(std::less<double>(),
	//						boost::bind(&RRTNode::shortestDistance,_1),
	//						boost::bind(&RRTNode::shortestDistance,_2) ) ) ) )->state;
	return GameStatePtr();

}

Pose RRTPlanner::choseTarget(Pose goalPose,Vector2D velocity,const Pose nearestPose, const double obstacleDist){
	Pose result;

	double t=uniformGen_01();
	//jesli w poprzednim kroku znaleziono sciezke do celu
	if(this->path->size()!=0){
		if(t<0.1){
			result=goalPose;
			//nearest=findNearestState(result);
			//nearest=findNearestToTargetState();
		}
		//way point
		else if (t<0.7){
			double p=1.0/this->path->size();
			double g=uniformGen_01();
			//wybieram jako docelowy jeden z wezlow ze sciezki znalezionej w poprzednim uruchomieniu RRT
			std::list<Pose>::iterator ii=this->path->begin();
			for(int i=1;ii!=this->path->end();ii++,i++){
				if(g<p*i){
					result=*ii;
					break;
				}
			}
		}
		else
			result=getRandomPose();
	}
	//jesli nie znaleziono wczesniej sciezki do celu
	else{
		if(t<this->toTargetLikelihood){
			result=goalPose;
		}
		else
			result=getRandomPose();
	}

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
	if( this->root->children.size() < 4 ){
		if(this->root->getMyRobotPos().distance(targetPose) < distance ){
			result=this->root;
			this->shortestDist=this->root->shortestDistance=this->root->getMyRobotPos().distance(targetPose);
		}
	}
	return result;
}
RRTNodePtr RRTPlanner::findNearest(const Pose & targetPose,RRTNodePtr currNode){
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
/*
RRTNodePtr RRTPlanner::findNearestToTarget(RRTNodePtr currNode){
	RRTNodePtr result;

	Pose robotPose=currNode->getMyRobotPos();
	if(currNode->shortestDstToTarget==std::numeric_limits<double>::infinity())
		currNode->shortestDstToTarget=robotPose.distance(this->goalPose);
	result=currNode;

	//wezel potomny najblizej celu
	RRTNodePtr nearest;

	BOOST_FOREACH(RRTNodePtr node,currNode->children){
		nearest=findNearestToTarget(node);
		if( nearest->shortestDstToTarget < currNode->shortestDstToTarget ){
			currNode->shortestDstToTarget=nearest->shortestDstToTarget;
			result=nearest;
		}
	}
	return result;
}

RRTNodePtr RRTPlanner::findNearestToTargetState(){
	//korzen tez bierze udział w poszukiwaniu najblizszego punktu
	RRTNodePtr result=this->root;
	double distance;
	this->shortestDist=distance=0;
	Pose robotPose=root->getMyRobotPos();
	if(this->root->shortestDstToTarget==std::numeric_limits<double>::infinity())
		this->root->shortestDstToTarget=robotPose.distance(this->goalPose);
	this->resultNode=this->root;

	//znajdz potomka najblizej celu
	BOOST_FOREACH(RRTNodePtr node,this->root->children){
		result=findNearestToTarget(node);
		if(result.get()!=NULL){
			robotPose=result->getMyRobotPos();
			distance=robotPose.distance(this->goalPose);
			this->shortestDist=distance;
		}
	}
	//sprawdz czy przypadkiem korzen nie jest blizej celu
	//ale unikaj nadmiernego rozgalezienia
	if( this->root->children.size() < 4 ){
		if(this->root->getMyRobotPos().distance(this->goalPose) < distance )
			result=this->root;
			this->shortestDist=this->root->getMyRobotPos().distance(this->goalPose);
	}

	return result;
}
*/
RRTNodePtr RRTPlanner::findNearestAttainableState(const Pose & targetPose){
	//korzen tez bierze udział w poszukiwaniu najblizszego punktu
	RRTNodePtr result;
	path->clear();
	//znajdz bezposrednio osiagalnego potomka najblizej celu
	BOOST_FOREACH(RRTNodePtr node,this->root->children){
		result=findNearestAttainableState(targetPose,node);
		//TODO: posprawdzac potomkow korzenia pod wzgledem osiagalnosci o odleglosci
	}
	return result;
}
RRTNodePtr RRTPlanner::findNearestAttainableState(const Pose & targetPose,RRTNodePtr currNode){
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
	//Pose robotPose=currentState.get()->getRobotPos(this->robotName);
	static boost::mt19937 rngX(static_cast<unsigned> (time(NULL)));
	static boost::mt19937 rngY(static_cast<unsigned> (2*time(NULL)));

	//generator wspolrzednej X
	//static boost::uniform_real<double> uni_distX(0,4);
	//double maxXvalue=5.4;
	double maxXvalue=5;
	//interesujace sa jedynie pozycje rozniace sie co najwyzej o 0.01 [m]
	static boost::uniform_int<int> uni_distX(0,maxXvalue*100);

//	static boost::variate_generator<boost::mt19937, boost::uniform_real<double> >
	static boost::variate_generator<boost::mt19937, boost::uniform_int<int> >
	genX(rngX, uni_distX);

	//generator wspolrzednej Y
//	static boost::uniform_real<double> uni_distY(0,4);
//	double maxYvalue=7.4;
	double maxYvalue=7;
	//interesujace sa jedynie pozycje rozniace sie co najwyzej o 0.01 [m]
	static boost::uniform_int<int> uni_distY(0,maxYvalue*100);

	//static boost::variate_generator<boost::mt19937, boost::uniform_real<double> >
	static boost::variate_generator<boost::mt19937, boost::uniform_int<int> >
		genY(rngY, uni_distY);
	double x=genX();
	double y=genY();
	Pose randomPose(x/100.0,y/100.0,0.0);

	return randomPose;
}

Pose RRTPlanner::getRandomPose(Pose currentPose){
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
	std::ostringstream ois;
	static double deltaT=0.1;//0.1  sekund
	Vector2D v1=Vector2D (currentVel.x+deltaVel,currentVel.y-deltaVel);
	Vector2D v2=Vector2D (currentVel.x-deltaVel,currentVel.y+deltaVel);

	ois<<"current Pose "<<currentPose<<" currentVel"<<currentVel<<" v1 "<<v1<<" v2 "<<v2<<std::endl;
	Logger::getInstance().LogToFile(DBG,ois);
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
	Logger::getInstance().LogToFile(DBG,ois);
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
	Logger::getInstance().LogToFile(DBG,ois);
	boost::triangle_distribution<double> distY(a, c , b);
	boost::variate_generator<boost::mt19937&, boost::triangle_distribution<double> >  genY(rng, distY);

	double x=genX();
	double y=genY();

	Pose randomPose(x,y,0.0);
	return randomPose;
}

GameStatePtr RRTPlanner::extendState(const GameStatePtr & currState,const Pose& targetPose,const double robotReach_){

	assert(currState.get()!=NULL);
	double robotReach=robotReach_;
	//sprawdzenie czy targetPose nie lezy w obrebie przeszkody
	if(!isTargetInsideObstacle( targetPose, SAFETY_MARGIN ,true) ){
		return GameStatePtr();
	}
	std::ostringstream log;

	GameStatePtr result( new GameState(*currState));
	Pose currPose=currState->getRobotPos(this->robotName);

	//wyznacz rownanie prostej laczacej stan biezacy z docelowym
	double x=0,y=0;
	//jesli cel jest blizej niz niz RRT::RobotReach
	if(fabs(targetPose.distance(currPose) )< robotReach){
		 robotReach=fabs(targetPose.distance(currPose) );
	}
	//jesli jest to prosta typu x=A
	if( fabs(currPose.get<0>()-targetPose.get<0>() ) < 0.001 ){
		result->updateRobotData(this->robotName,
			Pose(currPose.get<0>(),currPose.get<1>()+robotReach,currPose.get<2>()),
					currState->getRobotVelocity(this->robotName)
			);
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
		result->updateRobotData(this->robotName,
					Pose(x,y,currPose.get<2>()) );
	}

	Pose resultPose=result->getRobotPos(this->robotName);
	double test=pow(resultPose.get<0>()-currPose.get<0>(),2)+pow(resultPose.get<1>()-currPose.get<1>(),2);
	//test poprawnosci rozwiazania
	assert(fabs(pow(robotReach,2)-test)<0.001);

	//czy ten punkt nie lezy w obrebie przeszkody
	if(!isTargetInsideObstacle(  resultPose ,SAFETY_MARGIN,true) ){
		return GameStatePtr();
	}

	double angle=currState->getRobotVelocity(this->robotName).angleTo(Vector2D(x,y));

	result->updateRobotData(this->robotName,
			Pose(x,y,currPose.get<2>()),
					currState->getRobotVelocity(this->robotName).rotate(angle) );

	return result;
}
void RRTPlanner::evaluateEnemyPositions(const GameStatePtr & currState,const double deltaSimTime){
	const std::vector<std::string> blueTeam=Config::getInstance().getBlueTeam();

	BOOST_FOREACH(std::string modelName,blueTeam){
		if(modelName.compare(this->robotName)!=0){
			Vector2D v=(*currState).getRobotVelocity(modelName);
			Pose rPose=(*currState).getRobotPos(modelName);
			Pose newPose(rPose.get<0>()+v.x*deltaSimTime,rPose.get<1>()+v.y*deltaSimTime,0);
			this->predictedObstaclesPos.push_back(newPose);
			//std::cout<<"add obstacle blue "<<nr++<<" robot velocity "<<v<<std::endl;
			//(*currState).updateRobotData(modelName,newPose,v,0);
		}
	}

	const std::vector<std::string> redTeam=Config::getInstance().getRedTeam();
	BOOST_FOREACH(std::string modelName,redTeam){
		if(modelName.compare(this->robotName)!=0){
			Vector2D v=(*currState).getRobotVelocity(modelName);
			Pose rPose=(*currState).getRobotPos(modelName);
			Pose newPose(rPose.get<0>()+v.x*deltaSimTime,rPose.get<1>()+v.y*deltaSimTime,0);
			this->predictedObstaclesPos.push_back(newPose);
			//std::cout<<"add obstacle red "<<nr++<<" robot velocity "<<v<<std::endl;
			//(*currState).updateRobotData(modelName,newPose,v,0);
		}
	}
}

bool RRTPlanner::checkTargetAttainability(const Pose &currPose,const Pose &targetPose,bool checkAddObstacles){

	double robotRadius=Config::getInstance().getRRTRobotRadius()+SAFETY_MARGIN;
	Vector2D tar=( Vector2D( targetPose.get<0>(),targetPose.get<1>() ) -
			Vector2D( currPose.get<0>(),currPose.get<1>() ) );
	//transformacja do lokalnego ukl wspolrzednych
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
	std::cout<<"pozycja celu w ukl zw z robotem "<<target<<std::endl;
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
		//std::cout<<"obstacle position in robot coordinates "<<obstaclePosition<<std::endl;
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
			std::cout<<"obstacle position in robot coordinates "<<obstaclePosition<<std::endl;
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
						pow(robotRadius,2) ){
			//std::cout<<"collosion with "<<obstaclePose<<std::endl;
			return false;
		}
	}

	if(checkAddObstacles){
		//sprawdz czy punkt docelowy nie lezy w obrebie przewidywanego polozenia przeszkody
		BOOST_FOREACH(Pose obstaclePose,this->predictedObstaclesPos){
			if(  pow(targetPose.get<0>()-obstaclePose.get<0>(),2) + pow(targetPose.get<1>()-obstaclePose.get<1>(),2) <=
							pow(robotRadius,2) ){
				return false;
			}
		}
	}
	return true;
}

double RRTPlanner::distanceToNearestObstacle(const GameStatePtr & currState,const Pose &currPose){
	Pose obstaclePose;
	const std::vector<std::string> blueTeam=Config::getInstance().getBlueTeam();
	const std::vector<std::string> redTeam=Config::getInstance().getBlueTeam();

	std::ostringstream log;
	double distance=numeric_limits<double>::max();

	BOOST_FOREACH(std::string modelName,blueTeam){
		if(modelName.compare(this->robotName)!=0){
			obstaclePose=currState->getRobotPos(modelName);
			distance=std::min<double>(distance,currPose.distance(obstaclePose));
		}
	}

	BOOST_FOREACH(std::string modelName,redTeam){
			if(modelName.compare(this->robotName)!=0){
				obstaclePose=currState->getRobotPos(modelName);
				distance=std::min<double>(distance,currPose.distance(obstaclePose));
			}
		}
	return distance;
}

int RRTPlanner::serializeTree(const char * fileName,int serializedTrees){
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

RRTPlanner::~RRTPlanner() {
	ofstream myfile;
	myfile.open ("rrtTree.xml", ios::out | ios::app);
	myfile<<std::endl;
	myfile.close();
}
