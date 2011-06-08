/*
 * GameState.cpp
 *
 *  Created on: 2009-05-12
 *      Author: Maciej Gąbka
 */

#include "GameState.h"
#include "../RotationMatrix/RotationMatrix.h"
#include <boost/foreach.hpp>

GameState::GameState(): redGoal(top), blueGoal(bottom) {
//	updated=false;
	this->simTime = 0;
}

GameState::GameState(const GameState& gameState): redGoal(gameState.redGoal),
			blueGoal(gameState.blueGoal){

//	this->updated=false;
	this->ball=gameState.ball;
	this->robots=gameState.robots;
	this->simTime=gameState.simTime;
}

void GameState::updateRobotData(std::string name,Pose pos,Vector2D v,double w){
	this->robots[ Robot::getRobotID(name) ]=RobotState(pos,v,w);
}

void GameState::updateRobotData(Robot::robotID id,Pose pos,Vector2D relRobotVel,double w){

	this->robots[id]=RobotState(pos,relRobotVel,w);
}

void GameState::updateRobotVel(std::string name,std::pair<Vector2D,double> vel){

	this->robots[Robot::getRobotID(name)].v=vel.first;
	this->robots[Robot::getRobotID(name)].w=vel.second;
}

void GameState::updateRobotVel(Robot::robotID id,std::pair<Vector2D,double> vel){

	this->robots[id].v=vel.first;
	this->robots[id].w=vel.second;
}

void GameState::updateBallData(Vector2D pos,Vector2D v){
	this->ball=Ball(pos,v);
}
void GameState::updateSimTime(double simTime){
	this->simTime=simTime;
}
std::vector<Pose> GameState::getEnemyRobotsPos(const Robot::robotID & id) const {
	std::vector<Pose> result;
	RobotsPoseConstIt ii=this->robots.begin();
	for(;ii!=this->robots.end();ii++){
		if( ii->first != id)
			result.push_back(ii->second.pos);
	}
	return result;
}
/*
Pose GameState::getRobotPos(std::string name){
	return this->robots[name].pos;
}

Vector2D GameState::getRobotVelocity(std::string name){
	return this->robots[name].v;
}
*/
Pose GameState::getRobotPos(const Robot::robotID id) const {
	RobotsPoseConstIt  r = robots.find(id);
	if(r == robots.end()){
		std::cout<<"looking for "<<id<<" game state "<<this<<std::endl;
	}
	assert( r != robots.end() );
	return r->second.pos;
	// return robots[id].pos;
}

Vector2D GameState::getRobotGlobalVelocity(const Robot::robotID id) const {
	RobotsPoseConstIt  r = robots.find(id);
	assert( r != robots.end() );
	return r->second.v;
	//return this->robots[id].v;
}

Vector2D GameState::getBallGlobalVelocity( ) const {
	return this->ball.v;
}

double GameState::getRobotAngularVelocity(const  Robot::robotID id) const {
	RobotsPoseConstIt  r = robots.find(id);
	assert( r != robots.end() );
	return r->second.w;
	//return this->robots[id].w;
}
Pose GameState::getBallPos() const {
	return Pose(this->ball.pos.x,this->ball.pos.y,0.0);
}
double GameState::getSimTime() const {
	return this->simTime;
}

GameState & GameState::operator=(const GameState &gameState){
	///dane dotyczace robotow
	this->robots=gameState.robots;
	if( this->robots.size() != 6 ){
		std::cout<<"warning this->robots.size() != 6 gameState "<<*this<<std::endl;
	}
	assert( this->robots.size() == 6 );
	///dane dotyczace pilki
	this->ball=gameState.ball;
	this->simTime=gameState.simTime;
	return *this;
}

std::ostream& operator<<(std::ostream& os,const GameState& gs){

	std::pair<Robot::robotID,GameState::RobotState> robot;
	os<<"simTime "<<gs.simTime<<" ";
	BOOST_FOREACH(robot, gs.robots){
		//os<<robot.second<<"\n";
		os<<robot.first<<" "<<robot.second<<"\n";
	}
	return os;
}

std::ostream& operator<<(std::ostream& os,const GameState* const gs) {

	std::pair<Robot::robotID,GameState::RobotState> robot;
	os<<"simTime "<<gs->simTime<<" ";
	BOOST_FOREACH(robot, gs->robots){
		//os<<robot.second<<"\n";
		os<<robot.first<<" "<<robot.second<<"\n";
	}
	return os;
}

std::ostream& operator<<(std::ostream& os,const GameState::RobotState& robot){

	os<<"robot position "<<robot.pos<<" linear velocity="<<robot.v<<" angular velocity="<<robot.w;
	return os;
}
GameState::~GameState() {
	// TODO Auto-generated destructor stub
}
