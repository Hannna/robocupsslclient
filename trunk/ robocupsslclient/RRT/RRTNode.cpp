/*
 * RRTNode.cpp
 *
 *  Created on: 2010-01-12
 *      Author: maciek
 */

#include "RRTNode.h"
#include <limits>
#include <sstream>

#include <boost/foreach.hpp>
#include "../Config/Config.h"

std::ostream& operator<<(std::ostream& os,const RRTNode& node){
	os<<"robot pose"<<node.currRobotPose;
	return os;
}


RRTNode::RRTNode(const GameStatePtr state_,const Robot::robotID & id, bool root):
                        state(state_),  currRobotPose( (*state).getRobotPos( id ) ), isRoot(root){
	this->final=false;
	shortestDistance=std::numeric_limits<double>::infinity();
//	this->shortestDstToTarget=std::numeric_limits<double>::infinity();
};
void RRTNode::setFinal(){
	this->final=true;
}

void RRTNode::addNode(const RRTNodePtr & node){
	children.push_back(node);
}
Pose RRTNode::getRobotPos( const Robot::robotID & id ) const {
	return state.get()->getRobotPos( id );
}
const Pose RRTNode::getMyRobotPos()const{
	return currRobotPose;
}
int RRTNode::serializeNodeToXml( xmlTextWriterPtr & writer){
	//zapisz polozenie wszystkich robotow
	const std::vector<std::string> blueTeam=Config::getInstance().getBlueTeam();
	const std::vector<std::string> redTeam=Config::getInstance().getRedTeam();
	int status;
	status = xmlTextWriterStartElement(writer, BAD_CAST "BlueTeam");
	if (status < 0) {
		printf("testXmlwriterTree: Error at xmlTextWriterStartElement\n");
		return status;
	}

	serializeTeamToXml(writer,blueTeam,"blue");

	// Close the element named Blue Team
    status = xmlTextWriterEndElement(writer);
    if (status < 0) {
        printf("testXmlwriterTree: Error at xmlTextWriterEndElement in serializeTree\n");
        return status;
    }

	status = xmlTextWriterStartElement(writer, BAD_CAST "RedTeam");
	if (status < 0) {
		printf("testXmlwriterTree: Error at xmlTextWriterStartElement\n");
		return status;
	}
	serializeTeamToXml(writer,redTeam,"red");

	// Close the element named Red Team
    status = xmlTextWriterEndElement(writer);
    if (status < 0) {
        printf("testXmlwriterTree: Error at xmlTextWriterEndElement in serializeTree\n");
        return status;
    }
    return 0;
}

int serializeRobotToXml(xmlTextWriterPtr & writer,const char * eltname,Pose robotPose,int eltNr)
{
	std::ostringstream ois;
	//Pose robotPose=this->state->getRobotPos(modelName);
	//std::cout<<"node name Node"<<std::endl;
	int status;
	status = xmlTextWriterStartElement(writer, BAD_CAST eltname);

	if (status < 0) {
		printf("testXmlwriterTree: Error at xmlTextWriterStartElement\n");
		return status;
	}

	ois<<eltNr;
	status = xmlTextWriterWriteAttribute(writer, BAD_CAST "nr",
									 BAD_CAST ois.str().c_str());
	if (status < 0) {
		printf("testXmlwriterTree: Error at xmlTextWriterWriteAttribute\n");
		return status;
	}

	ois.str("");
	ois<<robotPose.get<0>();
	status = xmlTextWriterWriteAttribute(writer, BAD_CAST "x",
									 BAD_CAST ois.str().c_str());
	if (status < 0) {
		printf("testXmlwriterTree: Error at xmlTextWriterWriteAttribute\n");
		return status;
	}
	ois.str("");
	ois<<robotPose.get<1>();
	status = xmlTextWriterWriteAttribute(writer, BAD_CAST "y",
									 BAD_CAST ois.str().c_str());
	if (status < 0) {
		printf("testXmlwriterTree: Error at xmlTextWriterWriteAttribute\n");
		return status;
	}

	ois.str("");
	ois<<Config::getInstance().getRRTRobotRadius();
	status = xmlTextWriterWriteAttribute(writer, BAD_CAST "radius",
									 BAD_CAST ois.str().c_str());
	if (status < 0) {
		printf("testXmlwriterTree: Error at xmlTextWriterWriteAttribute\n");
		return status;
	}

	status = xmlTextWriterEndElement(writer);
	if (status < 0) {
		printf("testXmlwriterTree: Error at xmlTextWriterWriteAttribute\n");
		return status;
	}
	ois.str("");
	return 0;
}
int RRTNode::serializeTeamToXml(xmlTextWriterPtr & writer,const std::vector<std::string> & team,const char * color){
	int playerNr=0;
	std::ostringstream ois;
	BOOST_FOREACH(std::string modelName,team){
		Pose robotPose=this->state->getRobotPos(Robot::getRobotID(modelName));
		serializeRobotToXml(writer,color,robotPose,playerNr++);
		//std::cout<<"node name Node"<<std::endl;
		/*
		int status;
		status = xmlTextWriterStartElement(writer, BAD_CAST color);

		if (status < 0) {
			printf("testXmlwriterTree: Error at xmlTextWriterStartElement\n");
			return status;
		}

		ois<<playerNr++;
		status = xmlTextWriterWriteAttribute(writer, BAD_CAST "nr",
										 BAD_CAST ois.str().c_str());
		if (status < 0) {
			printf("testXmlwriterTree: Error at xmlTextWriterWriteAttribute\n");
			return status;
		}

		ois.str("");
		ois<<robotPose.get<0>();
		status = xmlTextWriterWriteAttribute(writer, BAD_CAST "x",
										 BAD_CAST ois.str().c_str());
		if (status < 0) {
			printf("testXmlwriterTree: Error at xmlTextWriterWriteAttribute\n");
			return status;
		}
		ois.str("");
		ois<<robotPose.get<1>();
		status = xmlTextWriterWriteAttribute(writer, BAD_CAST "y",
										 BAD_CAST ois.str().c_str());
		if (status < 0) {
			printf("testXmlwriterTree: Error at xmlTextWriterWriteAttribute\n");
			return status;
		}

		ois.str("");
		ois<<Config::getInstance().getRRTRobotRadius();
		status = xmlTextWriterWriteAttribute(writer, BAD_CAST "radius",
										 BAD_CAST ois.str().c_str());
		if (status < 0) {
			printf("testXmlwriterTree: Error at xmlTextWriterWriteAttribute\n");
			return status;
		}

		status = xmlTextWriterEndElement(writer);
		if (status < 0) {
			printf("testXmlwriterTree: Error at xmlTextWriterWriteAttribute\n");
			return status;
		}
		ois.str("");*/
	}
	return 0;
}
int RRTNode::serializeRecursiveToXml( xmlTextWriterPtr & writer,const std::string & modelName){
	int status;
	std::ostringstream ois;
	Pose robotPose=this->state->getRobotPos( Robot::getRobotID(modelName) );
	//std::cout<<"node name Node"<<std::endl;

	status = xmlTextWriterStartElement(writer, BAD_CAST "Node");

	if (status < 0) {
		printf("testXmlwriterTree: Error at xmlTextWriterStartElement\n");
		return status;
	}

	if(this->final){
		ois<<1;
		status = xmlTextWriterWriteAttribute(writer, BAD_CAST "final",
											 BAD_CAST ois.str().c_str());
		if (status < 0) {
			printf("testXmlwriterTree: Error at xmlTextWriterWriteAttribute x \n");
			return status;
		}
	}

	ois.str("");
	ois<<robotPose.get<0>();
	status = xmlTextWriterWriteAttribute(writer, BAD_CAST "x",
									 BAD_CAST ois.str().c_str());
	if (status < 0) {
		printf("testXmlwriterTree: Error at xmlTextWriterWriteAttribute x \n");
		return status;
	}
	ois.str("");
	ois<<robotPose.get<1>();
	/* Add an attribute with name "xml:lang" and value "de" to ORDER. */
	status = xmlTextWriterWriteAttribute(writer, BAD_CAST "y",
									 BAD_CAST ois.str().c_str());
	if (status < 0) {
		printf("testXmlwriterTree: Error at xmlTextWriterWriteAttribute y \n");
		return status;
	}

	//zapisywanie predkosci robota w danym wezle

	ois.str("");
	ois<<state->getRobotGlobalVelocity( Robot::getRobotID(modelName) ).x;
	/* Add an attribute with name "xml:lang" and value "de" to ORDER. */
	status = xmlTextWriterWriteAttribute(writer, BAD_CAST "vx",
									 BAD_CAST ois.str().c_str());
	if (status < 0) {
		printf("testXmlwriterTree: Error at xmlTextWriterWriteAttribute vx \n");
		return status;
	}


	ois.str("");
	state->getRobotGlobalVelocity( Robot::getRobotID(modelName) );
	ois<<state->getRobotGlobalVelocity( Robot::getRobotID(modelName) ).y;
	/* Add an attribute with name "xml:lang" and value "de" to ORDER. */
	status = xmlTextWriterWriteAttribute(writer, BAD_CAST "vy",
									 BAD_CAST ois.str().c_str());
	if (status < 0) {
		printf("testXmlwriterTree: Error at xmlTextWriterWriteAttribute vy \n");
		return status;
	}

	//zapisywanie celu do ktorego ma prowadzic dany wezel

	ois.str("");
	ois<<this->target.get<0>();
	/* Add an attribute with name "xml:lang" and value "de" to ORDER. */
	status = xmlTextWriterWriteAttribute(writer, BAD_CAST "tx",
									 BAD_CAST ois.str().c_str());
	if (status < 0) {
		printf("testXmlwriterTree: Error at xmlTextWriterWriteAttribute tx \n");
		return status;
	}


	ois.str("");
	ois<<this->target.get<1>();
	/* Add an attribute with name "xml:lang" and value "de" to ORDER. */
	status = xmlTextWriterWriteAttribute(writer, BAD_CAST "ty",
									 BAD_CAST ois.str().c_str());
	if (status < 0) {
		printf("testXmlwriterTree: Error at xmlTextWriterWriteAttribute ty\n");
		return status;
	}


	if(!this->children.empty()){
		status = xmlTextWriterStartElement(writer, BAD_CAST "Nodes");
		if (status < 0) {
			printf("testXmlwriterTree: Error at xmlTextWriterStartElement\n");
			return status;
		}
	}

	BOOST_FOREACH(RRTNodePtr & node,this->children){
		node->serializeRecursiveToXml(writer,modelName);
	}
	//jesli ma elt Nodes
	if(!this->children.empty())
		status = xmlTextWriterEndElement(writer);

	//Node
	status = xmlTextWriterEndElement(writer);
    if (status < 0) {
        printf("testXmlwriterTree: Error at xmlTextWriterEndElement\n");
        return status;
    }

	return 0;
}
// pobierz pozycje docelowa do ktorej prowadzi dany wezel
const Pose RRTNode::getTargetPose() const{
	return this->target;
}
// ustaw pozycje docelowa do ktorej prowadzi dany wezel
void RRTNode::setTargetPose(const Pose & p){
	this->target=p;
}
const GameStatePtr RRTNode::getGameState(){
	return this->state;
}
RRTNode::~RRTNode() {
	// TODO Auto-generated destructor stub
}
