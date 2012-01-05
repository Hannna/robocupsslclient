/*
 * GoToPose.h
 *
 *  Created on: 2010-06-07
 *      Author: maciek
 */

#ifndef GOTOPOSE_H_
#define GOTOPOSE_H_

#include "Task.h"
class RRTPlanner;

class GoToPose: public Task {
public:
	/*
	 * tworzy obiekt kierujacy robota do pozycji position i stara sie aby robot byl zwrocony w strone wlasnie tej pozycji
	 */
	GoToPose(const Vector2D & position,Robot * robot, double maxDistToGoal = Config::getInstance().getRRTMinDistance() );
	/*
	 * tworzy obiekt kierujacy robota do pozycji position. w punkcie docelowym robot powinien miec rotacje okreslona przez rotation
	 */
	GoToPose(const Vector2D & position, const double rotation,  Robot * robot, double maxDistToGoal = Config::getInstance().getRRTMinDistance() );
	GoToPose(const Vector2D & position,Robot * robot, bool force, double maxDistToGoal = Config::getInstance().getRRTMinDistance());

	void addXConstraint(  std::pair<double, double> *xConstraints ){
		this->xConstraints = xConstraints;
		//LOG_INFO( log,"dodano ograniczenia x:["<<xConstraints->first <<";"<< xConstraints->second<<"]" );
	}

	void addYConstraint( std::pair<double, double> *yConstraints ){
		this->yConstraints = yConstraints;
		//LOG_INFO( log,"dodano ograniczenia y:["<<yConstraints->first <<";"<< yConstraints->second<<"]" );
	}

	void wayPointsDisabled(){
		way_points_disabled = true;
	}

	//czas w ms wszystkich uruchomien rrt w danym tasku
	double total_rrt_time;
	//liczba uruchomien rrt w danym tasku
	unsigned int rrt_iterations;
	unsigned int max_path_size;
	unsigned int min_path_size;
	unsigned int max_tree_size;
	unsigned int min_tree_size;

	virtual Task* nextTask();
	virtual ~GoToPose();
protected:
	virtual Task::status run(void * arg, int steps=-1);
private:
	GoToPose();
	GoToPose(const GoToPose &);
	GoToPose& operator=(const GoToPose&);

	std::pair<double, double> *xConstraints;
	std::pair<double, double> *yConstraints;

	RRTPlanner * rrt;
	std::list<Pose>  path;
	bool way_points_disabled;



	const Vector2D goalPose;
	bool serialize;
	const double maxDistToGoal;
	const double rotation;
	const bool spec_rot;
	/*
	 * obliczane przy uruchomieniu rrt
	 odleglosc punktu startowego od najblizszej przeszkody, brane sa pod uwage jedynie biezace polozenia robotow, jesli robot jest blizej przeszkody
	 to wywolywana jest funkcja powodujaca rozproszenie
	 */
	static const double minDistFromObstacle = 0.02;

	double currSimTime;
	double lastSimTime;
	const bool force;
};

#endif /* GOTOPOSE_H_ */
