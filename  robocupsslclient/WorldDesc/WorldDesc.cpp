#include "WorldDesc.h"

WorldDesc::WorldDesc()
{
}

WorldDesc::~WorldDesc()
{
}

void WorldDesc::addObject(double x,double y,double rot,std::string  name)
{
	this->objectsPos.push_back(boost::tuple<Vector2D,double,std::string>(Vector2D(x,y),rot,name));
}

void WorldDesc::addSpeedOrder(std::string modelName, double vl, double vr)
{
	this->speeds.push_back(std::pair<std::string, Vector2D>(modelName, Vector2D(vl,vr)));
}

pos2D WorldDesc::getObjPos(std::string name)
{
	pos2D result;
	std::vector<boost::tuple<Vector2D,double, std::string> >::iterator ii=this->objectsPos.begin();
	for(;ii!=this->objectsPos.end();ii++){
		if((*ii).get<2>()==name){
			result=pos2D((*ii).get<0>(),(*ii).get<1>());
			return result;
		}
	}
	return result;
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
	std::string name = boost::get<2>(this->objectsPos.front());
	return name;
}



void WorldDesc::display()
{
	std::cout<<"Nazwa świata: "+name+"\n";
	std::cout<<"TimeLimit: "<<timeLimit<<"\n";
	std::vector<boost::tuple<Vector2D,double, std::string> >::iterator ii=this->objectsPos.begin();
	for(;ii!=objectsPos.end();ii++){
		std::cout<<"\t"<<boost::get<2>(*ii)<<std::endl;
		std::cout<<"\t"<<boost::get<0>(*ii)<<std::endl;
		std::cout<<"\t"<<boost::get<1>(*ii)<<std::endl<<std::endl;
	}
	
	std::vector<std::pair<std::string, Vector2D> >::iterator si = this->speeds.begin();
	std::cout<<"Zadane predkosci:\n";
	for(;si!=speeds.end();si++){
		std::cout<<(*si).first<<" "<<(*si).second<<std::endl;
	}
}
