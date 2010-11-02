/*
 * RRTNode.h
 *
 *  Created on: 2010-01-12
 *      Author: maciek
 */

#ifndef RRTNODE_H_
#define RRTNODE_H_

#include <list>
#include <boost/shared_ptr.hpp>
#include "../GameState/GameState.h"
#include <libxml/encoding.h>
#include <libxml/xmlwriter.h>

class RRTNode;
typedef boost::shared_ptr <RRTNode>  RRTNodePtr;

class RRTNode {
	friend class RRTPlanner;
public:
	RRTNode(const GameStatePtr & state_,const std::string robotName);
	//void addNode(const GameStatePtr & state);
	void addNode(const RRTNodePtr & node);
	Pose getRobotPos(std::string robotName);
	const Pose getMyRobotPos() const ;
	// pobierz pozycje docelowa do ktorej prowadzi dany wezel
	Pose getTargetPose();
	// ustaw pozycje docelowa do ktorej prowadzi dany wezel
	void setTargetPose(const Pose & p);

	/*@brief zapisuje dany wezel do pliku xml
	 *
	 * @param [in] writer
	 * @param [in] modelName nazwa modelu ktorego pozycje zapisujemy
	 * @param [in] recursive czy  zapisujemy takze wszystkie dzieci
	 */
	int serializeRecursiveToXml( xmlTextWriterPtr & writer,const std::string & modelName);
	int serializeTeamToXml(xmlTextWriterPtr & writer,const std::vector<std::string> & team, const char * color);
	int serializeNodeToXml( xmlTextWriterPtr & writer);
	virtual ~RRTNode();
	void setFinal();
private:
	//stan planszy w aktualnym wezle
	GameStatePtr state;
	//punkt docelowy do ktorego ma prowadzic dany wezel
	Pose target;
	//aktualna pozycja robota w danym wezle
	const Pose currRobotPose;
	//odleglosc  calej sciezki do celu
	//rozumiana jako minimalna odleglosc od celu jakiegokolwiek wezła potomnego
	double shortestDistance;
	//double shortestDstToTarget;
	std::list<RRTNodePtr> children;
	//wezel do ktorego ma podazac robot w danym kroku.
	bool final;
};

int serializeRobotToXml(xmlTextWriterPtr & writer,const char * eltname,Pose robotPose,int eltNr);

#endif /* RRTNODE_H_ */
