
/*
 * Config.cpp
 *
 *  Created on: 2009-12-02
 *      Author: maciek
 */

#include "Config.h"
#include "../Logger/Logger.h"
#include <boost/regex.hpp>	//wyrażenia regularne - do interpretacji plików, -lboost_regex!!
#include <boost/lexical_cast.hpp>
#include <boost/tuple/tuple.hpp>
#include <string.h>

Config* Config::config;
pthread_mutex_t Config::mutex=PTHREAD_MUTEX_INITIALIZER;
bool Config::end = false;


Config::Config(): configFileName("config.xml"),log(getLoggerPtr("app_debug") ) {
	testMode=false;
	debug=false;
}
void Config::setTestMode(bool testMode){
	 this->testMode=testMode;
}
bool Config::isTestMode(){
	return this->testMode;
}

bool Config::isDebugMode(){
	return this->debug;
}

bool Config::load(std::string filename){

	bool status=true;
	std::cout<<"Wczytuje plik: "<<filename.c_str()<<std::endl;

	xmlDocPtr config;
	xmlNodePtr current;
	try{
		config = xmlParseFile(filename.c_str());

		if (config == NULL ) {
			std::string error("Brak pliku ");
			throw error +filename;
		}

		current = xmlDocGetRootElement(config);

		if (current == NULL) {
			std::string error("Błąd w pliku ");
			error+=filename + " pusty plik";
			xmlFreeDoc(config);
			throw error;
		}

		if (xmlStrcmp(current->name, (const xmlChar *) "config")) {
			std::string error("Root node!= config w pliku XML ");
			error+=filename;
			xmlFreeDoc(config);
			throw error;
		}

		current = current->xmlChildrenNode;
		while (current != NULL) {
			/*
			if(!xmlStrcmp(current->name,(const xmlChar *) "blueTeam")){
				//std::cout<<"blueTeam"<<std::endl;
				if(!loadBlueTeam(current, config)){
					LOG_DEBUG(log,"unable to load  blue team parameters");
					return false;
				}
			}
			if(!xmlStrcmp(current->name,(const xmlChar *) "redTeam")){
				//std::cout<<"redTeam"<<std::endl;
				if(!loadRedTeam(current, config)){
					LOG_DEBUG(log,"unable to load red team parameters");
					return false;
				}
			}
			*/
			if(!xmlStrcmp(current->name,(const xmlChar *) "settings")){
				//std::cout<<"settings"<<std::endl;
				if(!loadSettings(current, config)){
					LOG_DEBUG(log,"unable to load settings");
					return false;
				}
			}
			else if(!xmlStrcmp(current->name,(const xmlChar *) "RRT")){
				//std::cout<<"RRT"<<std::endl;
				if(!loadRRTCfg(current, config)){
					LOG_DEBUG(log,"unable to load RRT");
					return false;
				}
			}
			else if(!xmlStrcmp(current->name,(const xmlChar *) "robotParams")){
				if(!loadRobotParams(current, config)){
					LOG_DEBUG(log,"unable to load robot params");
					return false;
				}
			}
			current = current->next;
		}

		//loading robots name
		//std::list<std::string> robots = getRobotNames("/home/maciek/mgr/mgr/my_plansza/test_worlds/test_world_1.xml",
	    //		BAD_CAST "//*[namespace-uri()='http://playerstage.sourceforge.net/gazebo/xmlschema/#model']");

		std::list<std::string> robots = getRobotNames(this->worldName.c_str(),
	    		BAD_CAST "//*[namespace-uri()='http://playerstage.sourceforge.net/gazebo/xmlschema/#model']");


	    std::list<std::string>::iterator ii = robots.begin();

	    for(;ii!=robots.end();ii++){
	    	if(strncmp(ii->c_str(),"red",3)==0 ){
	    		this->redTeam.push_back(*ii);
	    	}
	    	else if (strncmp(ii->c_str(),"blue",3)==0 ){
	    	    this->blueTeam.push_back(*ii);
	    	}
	    }
	}
	catch(std::string& err){
		std::cout<<err<<std::endl;
	//	xmlCleanupParser();
		xmlFreeDoc(config);
		return false;
	}
	xmlFreeDoc(config);

	return status;
}

bool Config::loadTestMode(xmlNodePtr node,xmlDocPtr config){
	bool status=true;
	xmlNodePtr current = node->xmlChildrenNode;
	while (current != NULL) {
		if(!xmlStrcmp(current->name,(const xmlChar *) "modelName")){
			xmlChar * str;
			str = xmlNodeListGetString(config,current->xmlChildrenNode,1);
			testCfg.modelName = std::string((const char*) str);
			xmlFree(str);
		}
		else if(!xmlStrcmp(current->name,(const xmlChar *) "testTime")){
			xmlChar * str;
			str = xmlNodeListGetString(config,current->xmlChildrenNode,1);
			std::string data = std::string((const char*) str);
			from_string<time_t>(this->testCfg.testTime,data,std::dec);

			xmlFree(str);
		}
		else if(!xmlStrcmp(current->name,(const xmlChar *) "speedFactor")){
				xmlChar * str;
				str = xmlNodeListGetString(config,current->xmlChildrenNode,1);
				std::string data = std::string((const char*) str);
				from_string<double>(this->testCfg.speedFactor,data,std::dec);

				xmlFree(str);
			}
		else if(!xmlStrcmp(current->name,(const xmlChar *) "velocity")){
			xmlChar * str;
			str = xmlNodeListGetString(config,current->xmlChildrenNode,1);

			std::string vel = std::string((const char*) str);
			//size_t currPos=0,prevPos=0;
			double v[3];
			//std::cout<<vel<<" length "<<vel.length()<<std::endl;

			std::string t("([\\s]*)");
			std::string nr("([+-]?\\d+(\\.\\d+)?)"); //   opisuje liczbę float (dodatnią!)
			const boost::regex reg(t + nr + t + nr + t+ nr + t);
			boost::smatch what;
			if (boost::regex_match(vel, what, reg)) {
				v[0] = boost::lexical_cast<double>(what[2]);
				v[1] = boost::lexical_cast<double>(what[5]);
				v[2] = boost::lexical_cast<double>(what[8]);
			}
			LOG_TRACE(log,"**********");
			std::ostringstream ois;
			for(int i=0;i<3;i++){
				ois<<"v["<<i<<"]="<<v[i]<<"\t";
			}
			LOG_TRACE(log,ois.str().c_str());
			boost::tuple<double,double,double> p(v[0],v[1],v[2]);
			this->testCfg.velocities.push_back(p);
			xmlFree(str);
		}

		else if(!xmlStrcmp(current->name,(const xmlChar *) "position")){
			xmlChar * str;
			str = xmlNodeListGetString(config,current->xmlChildrenNode,1);

			std::string vel = std::string((const char*) str);
			//size_t currPos=0,prevPos=0;
			double pose[3];
			//std::cout<<vel<<" length "<<vel.length()<<std::endl;

			std::string t("([\\s]*)");
			std::string nr("([+-]?\\d+(\\.\\d+)?)"); //   opisuje liczbę float (dodatnią!)
			const boost::regex reg(t + nr + t + nr + t+ nr + t);
			boost::smatch what;
			if (boost::regex_match(vel, what, reg)) {
				pose[0] = boost::lexical_cast<double>(what[2]);
				pose[1] = boost::lexical_cast<double>(what[5]);
				pose[2] = boost::lexical_cast<double>(what[8]);
			}
			LOG_DEBUG(log,"**********");
			std::ostringstream ois;
			ois<<"x="<<pose[0]<<"\t"<<"y="<<pose[1]<<"\t"<<"rot="<<pose[2]<<"\t";
			LOG_DEBUG(log,ois.str().c_str());
			this->testCfg.positions.push_back(Pose(pose[0],pose[1],pose[2]));
			xmlFree(str);
		}
		current = current->next;
	}
	return status;
}

bool Config::loadSettings(xmlNodePtr node,xmlDocPtr config){
	bool status=true;
	xmlNodePtr current = node->xmlChildrenNode;
	while (current != NULL) {
		if(!xmlStrcmp(current->name,(const xmlChar *) "testMode")){
			//std::cout<<"testMode"<<std::endl;
			if(!loadTestMode(current,config))
				return false;
		}
		if(!xmlStrcmp(current->name,(const xmlChar *) "world")){
			xmlChar * str;
			str = xmlNodeListGetString(config,current->xmlChildrenNode,1);
			this->worldName = std::string((const char*) str);
			xmlFree(str);
		}
		if(!xmlStrcmp(current->name,(const xmlChar *) "debug")){
			xmlChar * str;
			str = xmlNodeListGetString(config,current->xmlChildrenNode,1);
			if(str!=NULL){
				std::string debug = std::string((const char*) str);
				if(!debug.compare("true")){
					this->debug=true;
					std::cout<<"debug "<<debug<<std::endl;
				}
			}
			xmlFree(str);
		}
		current = current->next;
	}
	return status;
}

bool Config::loadRobotParams(xmlNodePtr node,xmlDocPtr config){
	bool status=true;
	xmlNodePtr current = node->xmlChildrenNode;
	while (current != NULL) {
		if(!xmlStrcmp(current->name,(const xmlChar *) "mainCylinderRadius")){
			xmlChar * str;
			str = xmlNodeListGetString(config,current->xmlChildrenNode,1);
			std::string data = std::string((const char*) str);
			from_string<double>(this->robotParams.mainCylinderRadious,data,std::dec);
			xmlFree(str);
			//this->redTeam.push_back(data);
		}
		current = current->next;
	}
	return status;
}


bool Config::loadRedTeam(xmlNodePtr node,xmlDocPtr config){
	bool status=true;
	xmlNodePtr current = node->xmlChildrenNode;
	while (current != NULL) {
		if(!xmlStrcmp(current->name,(const xmlChar *) "modelName")){
			xmlChar * str;
			str = xmlNodeListGetString(config,current->xmlChildrenNode,1);
			std::string data = std::string((const char*) str);
			this->redTeam.push_back(data);
			xmlFree(str);
		}
		current = current->next;
	}
	return status;
}

bool Config::loadBlueTeam(xmlNodePtr node,xmlDocPtr config){
	bool status=true;
	xmlNodePtr current = node->xmlChildrenNode;
	while (current != NULL) {
		if(!xmlStrcmp(current->name,(const xmlChar *) "modelName")){
			xmlChar * str;
			str = xmlNodeListGetString(config,current->xmlChildrenNode,1);
			std::string data = std::string((const char*) str);
			this->blueTeam.push_back(data);
			xmlFree(str);
		}
		current = current->next;
	}
	return status;
}

bool Config::loadRRTCfg(xmlNodePtr node,xmlDocPtr config){
	bool status=true;
	xmlNodePtr current = node->xmlChildrenNode;
	while (current != NULL) {
		if(!xmlStrcmp(current->name,(const xmlChar *) "goalProb")){
			xmlChar * str;
			str = xmlNodeListGetString(config,current->xmlChildrenNode,1);
			std::string data = std::string((const char*) str);
			from_string<double>(this->rrtCfg.goalProb,data,std::dec);
			xmlFree(str);
		}
		else if(!xmlStrcmp(current->name,(const xmlChar *) "minDistance")){
			xmlChar * str;
			str = xmlNodeListGetString(config,current->xmlChildrenNode,1);
			std::string data = std::string((const char*) str);
			from_string<double>(this->rrtCfg.minDistance,data,std::dec);
			xmlFree(str);
		}
		else if(!xmlStrcmp(current->name,(const xmlChar *) "robotReach")){
			xmlChar * str;
			str = xmlNodeListGetString(config,current->xmlChildrenNode,1);
			std::string data = std::string((const char*) str);
			from_string<double>(this->rrtCfg.robotReach,data,std::dec);
			xmlFree(str);
		}
		else if(!xmlStrcmp(current->name,(const xmlChar *) "robotRadius")){
			xmlChar * str;
			str = xmlNodeListGetString(config,current->xmlChildrenNode,1);
			std::string data = std::string((const char*) str);
			from_string<double>(this->rrtCfg.robotRadius,data,std::dec);
			xmlFree(str);
			//std::cout<<"robotRadius"<<data<<std::endl;

			//exit(0);
		}
		else if(!xmlStrcmp(current->name,(const xmlChar *) "maxVelocity")){
			xmlChar * str;
			str = xmlNodeListGetString(config,current->xmlChildrenNode,1);
			std::string data = std::string((const char*) str);
			from_string<double>(this->rrtCfg.maxVel,data,std::dec);
			xmlFree(str);
		}
		else if(!xmlStrcmp(current->name,(const xmlChar *) "goalPosition")){
			xmlChar * str;
			str = xmlNodeListGetString(config,current->xmlChildrenNode,1);

			std::string vel = std::string((const char*) str);
			//pozycja docelowa jest pozycja pilki
			if(vel.compare("ball")==0){
				rrtCfg.goToBall=true;
			}
			else{
				rrtCfg.goToBall=false;
				double pose[3];
				//std::cout<<vel<<" length "<<vel.length()<<std::endl;

				std::string t("([\\s]*)");
				std::string nr("([+-]?\\d+(\\.\\d+)?)"); //   opisuje liczbę float (dodatnią!)
				const boost::regex reg(t + nr + t + nr + t+ nr + t);
				boost::smatch what;
				if (boost::regex_match(vel, what, reg)) {
					pose[0] = boost::lexical_cast<double>(what[2]);
					pose[1] = boost::lexical_cast<double>(what[5]);
					pose[2] = boost::lexical_cast<double>(what[8]);
				}
				LOG_DEBUG(log,"**********");
				std::ostringstream ois;
				ois<<"x="<<pose[0]<<"\t"<<"y="<<pose[1]<<"\t"<<"rot="<<pose[2]<<"\t";
				LOG_DEBUG(log,ois.str().c_str());
				this->rrtCfg.goalPose=Pose(pose[0],pose[1],pose[2]);
			}
			xmlFree(str);
		}
		current = current->next;
	}

	return status;
}
std::string Config::getTestModelName(){
	return this->testCfg.modelName;

}
time_t Config::getTestEstimatedTime(){
	return this->testCfg.testTime;
}
const double Config::getSpeedFactor() const{
	return this->testCfg.speedFactor;
}

double Config::getRobotMainCylinderRadious() const {
    return this->robotParams.mainCylinderRadious;
}

const  std::vector<boost::tuple<double,double,double> > Config::getVelTests()const {
	return this->testCfg.velocities;
}
const  std::vector<Pose > Config::getPoseTests()const {
	return this->testCfg.positions;
}
const  std::vector<std::string> Config::getBlueTeam()const {
	return this->blueTeam;
}
const  std::vector<std::string> Config::getRedTeam()const {
	return this->redTeam;
}

const double Config::getRRTMaxVel() const{
	return this->rrtCfg.maxVel;
}
const double Config::getRRTGoalProb() const{
	return this->rrtCfg.goalProb;
}
const double Config::getRRTMinDistance() const{
	return this->rrtCfg.minDistance;
}

const double Config::getRRTRobotReach() const{
	return this->rrtCfg.robotReach;
}

const double Config::getRRTRobotRadius() const{
	return this->rrtCfg.robotRadius;
}
const Pose Config::getRRTGoalPose() const{
	return this->rrtCfg.goalPose;
}
const bool Config::goToBall() const{
	return this->rrtCfg.goToBall;
}
Config::~Config() {
	//LOG_INFO(log,"destroy Config");
}
