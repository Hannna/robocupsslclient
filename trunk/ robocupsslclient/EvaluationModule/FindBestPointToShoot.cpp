/*
 * FindBestPointToShoot.cpp
 *
 *  Created on: Feb 4, 2012
 *      Author: maciek
 */

#include "FindBestPointToShoot.h"
//generator wspolrzednej X
boost::mt19937 FindBestPointToShoot::rngX(static_cast<unsigned> (time(NULL)));
//generator wspolrzednej Y
boost::mt19937 FindBestPointToShoot::rngY(static_cast<unsigned> (2*time(NULL)));

FindBestPointToShoot::FindBestPointToShoot(const Vector2D point_, const int distance_,
		const GameStatePtr& gamestate_,  const std::string robotName_,Robot::robotID rid_): point( point_ ), distance( distance_ ),
		gamestate(gamestate_), robotName( robotName_ ),rid(rid_), evaluation( EvaluationModule::getInstance() ){
	// TODO Auto-generated constructor stub
	//boost::uniform_int<int> uni_distX(0, distance);
	//genX= boost::variate_generator<boost::mt19937, boost::uniform_int<int> >(rngX, uni_distX);
	//boost::uniform_int<int> uni_distY( 0, distance);
	//genY= boost::variate_generator<boost::mt19937, boost::uniform_int<int> >(rngY, uni_distY);
}

std::pair<Vector2D, double> FindBestPointToShoot::bestTarget(){

	boost::uniform_int<int> uni_distX(-distance/2, distance/2);
	boost::variate_generator<boost::mt19937, boost::uniform_int<int> >genX(rngX, uni_distX);
	boost::uniform_int<int> uni_distY( -distance/2, distance/2);
	boost::variate_generator<boost::mt19937, boost::uniform_int<int> >genY(rngY, uni_distY);

	std::vector<Pose> obs = gamestate->getEnemyRobotsPos( this->rid );
	std::vector<Pose>::iterator ii;
	double bestScore = std::numeric_limits<double>::min();
	double angleToShoot;
	Vector2D bestAnswer = point;;
	for(int i=0;i<1000;i++){
		bool collision = false;
		double x=genX();
		double y=genY();

		double tx= x/1000.0;
		double ty = y/1000.0;
		Vector2D answer(point.x+tx,point.y+ty);

		std::vector<Pose>::iterator ii = obs.begin();
		for(ii = obs.begin() ;ii != obs.end();ii++){
			if(  pow(answer.x -ii->get<0>(),2) + pow(answer.y - ii->get<1>(),2) <= pow(Config::getInstance().getRRTRobotRadius(),2) ){
				collision = true;
			}
		}
		if(collision)
			continue;


		Vector2D v(0,0);
		Pose pos(answer,0);
		gamestate->updateRobotData(robotName,pos,v,0);
		double tmpAngleToShoot;
		double score = 0;
		std::pair<double, double> ang =evaluation.aimAtGoal( gamestate,robotName, tmpAngleToShoot,score);

		if(score > bestScore){
			bestScore = score;
			bestAnswer =  answer;
			angleToShoot = tmpAngleToShoot;
		}
	}

	return std::pair<Vector2D, double>(bestAnswer,angleToShoot);

}
FindBestPointToShoot::~FindBestPointToShoot() {
	// TODO Auto-generated destructor stub
}


;
