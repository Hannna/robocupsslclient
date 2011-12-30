#include "WorldDesc.h"

WorldDesc::WorldDesc()
{
	isDynamic = false;
	this->name = std::string("");

	/// limit czasu na przejazd w danej sytuacji
	this->timeLimit = 0;
}

WorldDesc::~WorldDesc()
{
}

void WorldDesc::addObject( double x, double y, double rot, std::string  name )
{
	this->objectsPositions.push_back( std::pair<Pose,std::string>( Pose(x,y,rot), name ) );
}

void WorldDesc::addSpeedOrder(std::string modelName, double vx, double vy)
{
	this->speeds.push_back(std::pair<std::string, Vector2D>(modelName, Vector2D(vx,vy)));
}

Pose WorldDesc::getObjPos(std::string name)
{
	std::vector< std::pair< Pose, std::string > >::iterator ii=this->objectsPositions.begin();
	for(;ii!=this->objectsPositions.end();ii++){
		if((*ii).second==name){
			return ii->first;
		}
	}
	return Pose(0,0,0);
}

void WorldDesc::setName(std::string name)
{
	this->name=name;
}

std::string WorldDesc::getName()
{
	return this->name=name;
}

///Ustawia maksymalny dopuszczalny czas trwania eksperymentu dla danego rozstawienia
void WorldDesc::setTimeLimit(double t)
{
	this->timeLimit = t;
}

double WorldDesc::getTimeLimit()
{
	return this->timeLimit;
}

std::string WorldDesc::getFirstRobotName()
{
	std::string name = this->objectsPositions.front().second;
	return name;
}



void WorldDesc::display()
{
	std::cout<<"Nazwa świata: "+name+"\n";
	std::cout<<"TimeLimit: "<<timeLimit<<"\n";
	std::vector< std::pair<Pose, std::string> >::iterator ii=this->objectsPositions.begin();
	for(;ii!=objectsPositions.end();ii++){
		std::cout<<"\t"<<ii->first.get<0>()<<std::endl;
		std::cout<<"\t"<<ii->first.get<1>()<<std::endl;
		std::cout<<"\t"<<ii->first.get<2>()<<std::endl<<std::endl;
	}
	
	std::vector<std::pair<std::string, Vector2D> >::iterator si = this->speeds.begin();
	std::cout<<"Zadane predkosci:\n";
	for(;si!=speeds.end();si++){
		std::cout<<(*si).first<<" "<<(*si).second<<std::endl;
	}
}
