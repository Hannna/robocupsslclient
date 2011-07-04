/*
 * Config.h
 *
 *  Created on: 2009-12-02
 *      Author: maciek
 */

#ifndef CONFIG_H_
#define CONFIG_H_


#include <cstring>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <libxml/xmlmemory.h>
#include <libxml/parser.h>
#include <boost/tuple/tuple.hpp>

#include <log4cxx/logger.h>
#include "../additional.h"
#include "../Lock/Lock.h"


class Config {
private:
	class RRTCfg{
	public:
		RRTCfg(){
			goalProb=0.5;
		};
		//prawdopodobienstwo kierowania robota na cel
		double goalProb;
		//promien okregu opisujacego przeszkode/robota
		double robotRadius;
		//odleglosc do jakiej planujemy sciezke za pomoca rrt
		double minDistance;
		//estymowana odleglosc jaka moze przemierzyc robot w trakcie 1 iteracji
		double robotReach;
		// pozycja docelowa
		Pose goalPose;
		bool goToBall;
		//maksymalna predkosc liniowa robota
		double maxVel;
	};
	class TestCfg{
	public:
		//czas trwania jednego testu
		time_t testTime;
		//nazwa testowanego modelu
		std::string modelName;
		//zestawy predkosci do kolejnych testow
		std::vector<boost::tuple<double,double,double> > velocities;
		std::vector<Pose > positions;
		//wspo skalujacy predkosci;
		double speedFactor;
	};

	class FieldParams{
		friend class Videoserver;
		public:
		const double FIELD_WIDTH;//[m]
		const double FIELD_LENGTH;//[m]
		//odleglosc miedzy linia koncowa boiska a koncem bandy
		const double FIELD_MARIGIN;//[m]
		const Vector2D FIELD_BOTTOM_LEFT_CORNER;
		const Vector2D FIELD_TOP_RIGHT_CORNER;
		const Pose FIELD_MIDDLE_POSE;
		const Vector2D FIELD_MIDDLE_VECTOR;
        const Vector2D GOAL_CORNER_LEFT_SHIFT;
		const Vector2D GOAL_CORNER_RIGHT_SHIFT;

		const Vector2D BOTTOM_GOAL_MID_POSITION;
		const Vector2D BOTTOM_GOAL_LEFT_CORNER;
		const Vector2D BOTTOM_GOAL_RIGHT_CORNER;
		const Vector2D TOP_GOAL_MID_POSITION;
		const Vector2D TOP_GOAL_LEFT_CORNER;
		const Vector2D TOP_GOAL_RIGHT_CORNER;

		const Region upperMiddleRegion;
		const Region upperLeftCornerRegion;
		const Region upperRightCornerRegion;
		const Region upperGoalFieldRegion;

		const Region bottomMiddleRegion;
		const Region bottomLeftCornerRegion;
		const Region bottomRightCornerRegion;
		const Region bottomGoalFieldRegion;




		public:
		inline FieldParams():FIELD_WIDTH( 5.4 ), FIELD_LENGTH( 7.4 ),FIELD_MARIGIN(0.675),
                FIELD_BOTTOM_LEFT_CORNER(0,0), FIELD_TOP_RIGHT_CORNER(5.4, 7.4),
				FIELD_MIDDLE_POSE ( 2.7, 3.7 ,0.0 ), FIELD_MIDDLE_VECTOR( 2.7,3.7 ),
				GOAL_CORNER_LEFT_SHIFT(-0.36, -0.1), GOAL_CORNER_RIGHT_SHIFT(0.36, -0.1),

				BOTTOM_GOAL_MID_POSITION( 2.7, 0.675 ),
				BOTTOM_GOAL_LEFT_CORNER( BOTTOM_GOAL_MID_POSITION + GOAL_CORNER_LEFT_SHIFT ),
				BOTTOM_GOAL_RIGHT_CORNER( BOTTOM_GOAL_MID_POSITION + GOAL_CORNER_RIGHT_SHIFT ),

				TOP_GOAL_MID_POSITION( 2.7 , 6.725 ),
				TOP_GOAL_LEFT_CORNER( TOP_GOAL_MID_POSITION + GOAL_CORNER_LEFT_SHIFT ),
				TOP_GOAL_RIGHT_CORNER( TOP_GOAL_MID_POSITION + GOAL_CORNER_RIGHT_SHIFT ),

				upperMiddleRegion( Vector2D( FIELD_MARIGIN, FIELD_MIDDLE_VECTOR.y ),
						Vector2D( FIELD_MARIGIN + FIELD_WIDTH, 0.75*FIELD_LENGTH ) ),

				upperLeftCornerRegion( Vector2D( FIELD_MARIGIN, 0.75*FIELD_LENGTH + FIELD_MARIGIN ),
								Vector2D( FIELD_MARIGIN + FIELD_WIDTH/3.0 ,FIELD_TOP_RIGHT_CORNER.y  ) ),

				upperRightCornerRegion( Vector2D( 0.5*FIELD_WIDTH + FIELD_MARIGIN , 0.75*FIELD_LENGTH + FIELD_MARIGIN ),
								FIELD_TOP_RIGHT_CORNER ),

				upperGoalFieldRegion( Vector2D( FIELD_MARIGIN + FIELD_WIDTH/3.0 , FIELD_MIDDLE_VECTOR.y  + FIELD_LENGTH/4.0),
								Vector2D( 0.66*FIELD_WIDTH + FIELD_MARIGIN , FIELD_LENGTH + FIELD_MARIGIN ) ),

				bottomMiddleRegion( Vector2D( FIELD_MARIGIN , FIELD_MARIGIN + 0.25*FIELD_LENGTH ),
						Vector2D( FIELD_MARIGIN + FIELD_WIDTH, FIELD_MARIGIN +  0.5*FIELD_LENGTH ) ),

				bottomLeftCornerRegion(  Vector2D( FIELD_MARIGIN,FIELD_MARIGIN ),
						Vector2D( FIELD_MARIGIN + 0.33*FIELD_WIDTH , FIELD_MARIGIN + 0.25* FIELD_LENGTH ) ),

				bottomRightCornerRegion( Vector2D( FIELD_MARIGIN + 0.66*FIELD_WIDTH, FIELD_MARIGIN ),
						Vector2D( FIELD_MARIGIN + FIELD_WIDTH, FIELD_MARIGIN + 0.25 * FIELD_LENGTH ) ),

				bottomGoalFieldRegion(Vector2D( FIELD_MARIGIN + 0.33*FIELD_WIDTH  , FIELD_MARIGIN ),
						Vector2D( FIELD_MARIGIN + 0.66*FIELD_WIDTH, FIELD_MARIGIN + 0.25*FIELD_LENGTH  ) )
				{
			/*
					std::cout<<"upperLeftCornerRegion "<<upperLeftCornerRegion<<std::endl;
					std::cout<<"upperRightCornerRegion "<<upperRightCornerRegion<<std::endl;
					std::cout<<"upperGoalFieldRegion "<<upperGoalFieldRegion<<std::endl;
					std::cout<<"upperMiddleRegion "<<upperMiddleRegion<<std::endl;

					std::cout<<"bottomLeftCornerRegion "<<bottomLeftCornerRegion<<std::endl;
					std::cout<<"bottomRightCornerRegion "<<bottomRightCornerRegion<<std::endl;
					std::cout<<"bottomGoalFieldRegion "<<bottomGoalFieldRegion<<std::endl;
					std::cout<<"bottomMiddleRegion "<<bottomMiddleRegion<<std::endl;
			*/
		}

	};

	class RobotParams{
	public:
        double mainCylinderRadious;
	};
	static Config* config;
	static pthread_mutex_t  mutex;

public:
	//wymiary boiska
	FieldParams field;
	static bool end;
	bool load(std::string configFileName);
    double getRobotMainCylinderRadious() const ;
	const std::vector<std::string> getBlueTeam()const;
	const std::vector<std::string> getRedTeam()const;
	std::string getTestModelName();
	bool isDebugMode();
	time_t getTestEstimatedTime();
	const double getSpeedFactor() const;
	const std::vector<boost::tuple<double,double,double> > getVelTests() const;
	const std::vector<Pose > getPoseTests() const;
	const double getRRTMaxVel() const;
	const double getRRTGoalProb() const;
	const double getRRTMinDistance() const;
	const double getRRTRobotReach() const;
	const double getRRTRobotRadius() const;
	const Pose getRRTGoalPose() const;
	const bool goToBall() const;
	bool isTestMode();
	void setTestMode(bool );

//	static Config& getInstance(){
//		static Config Config_;
//		return Config_;
//	}

	static Config& getInstance(){
		if(!Config::config){
			LockGuard lock(Config::mutex);
			if(!Config::config){
				static Config c;
				//Videoserver::video=new Videoserver();
				Config::config = &c;
			}
		}
		return *Config::config;
	}

	virtual ~Config();

private:
	Config();
	Config(const Config&);
	Config& operator=(const Config& );
	//konfiguracja pojedynczego testu
	TestCfg testCfg;
	//konfiguracja algorytmu planowania sciezki
	RRTCfg rrtCfg;
	//parametry bazy jezdnej robota
	RobotParams robotParams;
	bool testMode;
	//czy zapisujemy logi do plikow
	bool debug;
	//nazwy modeli druzyny niebieskiej
	std::vector<std::string> blueTeam;
	//nazwy modeli druzyny czerwonej
	std::vector<std::string> redTeam;
	const std::string configFileName;

	bool loadTestMode(xmlNodePtr node,xmlDocPtr config);
	bool loadSettings(xmlNodePtr node,xmlDocPtr config);
	bool loadBlueTeam(xmlNodePtr node,xmlDocPtr config);
	bool loadRedTeam(xmlNodePtr node,xmlDocPtr config);
	bool loadRRTCfg(xmlNodePtr node,xmlDocPtr config);
	bool loadRobotParams(xmlNodePtr node,xmlDocPtr config);

	log4cxx::LoggerPtr log;

};

#endif /* CONFIG_H_ */
