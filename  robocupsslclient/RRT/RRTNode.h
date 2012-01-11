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
	friend std::ostream& operator<<(std::ostream& os,const RRTNode& node);
public:
	RRTNode(const GameStatePtr state_,const Robot::robotID & id, bool root=false);
	//void addNode(const GameStatePtr & state);
	void addNode(const RRTNodePtr & node);
	Pose getRobotPos(const Robot::robotID & id) const;
	const GameStatePtr getGameState();
	const Pose getMyRobotPos() const ;
	// pobierz pozycje docelowa do ktorej prowadzi dany wezel
	const Pose getTargetPose() const;
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
	bool isRootNode(){
		return isRoot;
	}
private:
	//stan planszy w aktualnym wezle
	const GameStatePtr state;
	//punkt docelowy do ktorego ma prowadzic dany wezel
	Pose target;
	//pozycja robota (dla którego wyznaczana jest sciezka), w kroku algorytmu
	const Pose currRobotPose;
	//odleglosc  calej sciezki do celu
	//rozumiana jako minimalna odleglosc od celu jakiegokolwiek wezła potomnego
	double shortestDistance;
	const bool isRoot;

	std::list<RRTNodePtr> children;
	//czy jest to wezeł na ścieżce końcowej
	bool final;
};

int serializeRobotToXml(xmlTextWriterPtr & writer,const char * eltname,Pose robotPose,int eltNr);

#endif /* RRTNODE_H_ */
