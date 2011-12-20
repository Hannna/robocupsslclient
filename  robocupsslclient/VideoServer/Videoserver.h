#ifndef VIDEOSERVER_H_
#define VIDEOSERVER_H_

#include <boost/shared_ptr.hpp>
#include <boost/regex.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include "../additional.h"
#include "../SimControl/SimControl.h"
#include "../GameState/GameState.h"
#include "../Logger/Logger.h"
#include "../Thread/Thread.h"
#include "../Lock/Lock.h"
#include "../Logger/Logger.h"

/**
 *  @author Kamil Muszynski i Maciej Gąbka
 *  @brief klasa modelujaca funkcjonalnosc rzeczywistego programu
 *  wykorzystywanego do zwracania pozycji robotów w lidze robocub F-180
 *
 */

class Videoserver : public Thread
{

friend void updateVideo(int);
friend void update(int);

public:

	static Videoserver& getInstance(){

		if(!Videoserver::video){
			LockGuard lock(mutex);
			if(!Videoserver::video){
				static Videoserver v;
				//Videoserver::video=new Videoserver();
				Videoserver::video = &v;
			}
		}
		return *Videoserver::video;
	}

	/**
	 * metoda wypelniajaca zmienna data
	 * pozycjami oraz prędkościami obiektów na planszy - gracze + pilka
	 *
	 * @return zwraca czas symulatora z jakiego pochodza te pozycje
	 */
	double updateGameState(GameStatePtr& gameState) const;


	/**
	 * zwraca czas pomiedzy 2 ostatnimi aktualizacjami polozenia robotow
	 *
	 */
	double getUpdateDeltaTime() const;

	/**
	@brief rejestruje dany interfejs robota
	*/
	#ifdef GAZEBO
		#ifdef OLD
		void registerRobot( gazebo::PositionIface *posIface,std::string robotName);
		#else
		void registerRobot( libgazebo::PositionIface *posIface,std::string robotName);
		#endif
	#endif

    /**
    @brief testuje pobieranie pozycji oraz predkosci z symulatora
    */
    void testVideoserver();

    void stop();

	static goalArea redGoal;
	static goalArea blueGoal;

	static const Vector2D  getRedGoalMidPosition();
	static const Vector2D  getRedGoalLeftCornerPosition();
	static const Vector2D  getRedGoalRightCornerPosition();

	static const Vector2D  getBlueGoalMidPosition();
	static const Vector2D  getBlueGoalLeftCornerPosition();
	static const Vector2D  getBlueGoalRightCornerPosition();

//	static void setRedGoalMidPosition( const Vector2D & );
//	static void setRedGoalLeftCornerPosition( const Vector2D & );
//	static void setRedGoalRightCornerPosition( const Vector2D & );

//	static void setBlueGoalMidPosition( const Vector2D & );
//	static void setBlueGoalLeftCornerPosition( const Vector2D &);
//b	static void setBlueGoalRightCornerPosition( const Vector2D & );


//czas co jaki videoserwer pobiera inf z symulatora
	static const int updateDeltaTime=50000; //100[ms]
private:
	virtual void execute(void*) ;
	/*@brief pobiera z symulatora pozycje wszystkich robotow na planszy oraz pilki
	 *
	 */
	void update();
	GameStatePtr gameState;
	static Videoserver * video;
	Videoserver();
	Videoserver(const Videoserver& v);
	virtual ~Videoserver(){
		LOG_FATAL(log," try to destroy videoserver");
		pthread_mutex_lock (&Videoserver::mutex);
		//delete video;
		Videoserver::video = NULL;
		pthread_mutex_unlock (&Videoserver::mutex);

//		log4cxx::LogManager::getLoggerRepository()->shutdown();
		LOG_INFO(log,"destroy videoserver");
	};
	Videoserver& operator=(const Videoserver&);

	log4cxx::LoggerPtr log;
	///czas symulatora podczas wykonania ostatniej aktualizacji
	double lastUpdateTime; //[s]
	//odstep pomiedzy ostatnimi aktualizacjami
	double updateT;//[s]
	//mutex chroniacy dostep do singletonu
	static pthread_mutex_t  mutex;
	static pthread_cond_t update_game_state_cv;

	#ifdef GAZEBO
		#ifdef OLD
		std::map<std::string,gazebo::PositionIface * > posIfaces;
		typedef std::map<std::string,gazebo::PositionIface * >::iterator PosIfacesIterator;
		#else
		std::map<std::string,libgazebo::PositionIface * > posIfaces;
		typedef std::map<std::string,libgazebo::PositionIface * >::iterator PosIfacesIterator;
		#endif

	#endif
	static struct timeval startTime;

	static Vector2D redGoalMidPosition;
	static Vector2D redGoalLeftCornerPosition;
	static Vector2D redGoalRightCornerPosition;

	static Vector2D blueGoalMidPosition;
	static Vector2D blueGoalLeftCornerPosition;
	static Vector2D blueGoalRightCornerPosition;

	bool stopFlag;

	bool simError;
};


#endif /*VIDEOSERVER_H_*/
