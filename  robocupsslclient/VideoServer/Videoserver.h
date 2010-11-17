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

/**
 *  @author Kamil Muszynski i Maciej Gąbka
 *  @brief klasa modelujaca funkcjonalnosc rzeczywistego programu
 *  wykorzystywanego do zwracania pozycji robotów w lidze robocub F-180
 *
 */

class Videoserver : public Thread
{
friend void updateVideo(int);
public:
	static Videoserver& getInstance(){
        //pthread_mutexattr_t mutexAttribute;
        //pthread_mutexattr_init (&mutexAttribute);
        //pthread_mutexattr_settype(&mutexAttribute,
        //PTHREAD_MUTEX_RECURSIVE);
        //pthread_mutex_init (&Videoserver::mutex, &mutexAttribute);

       // pthread_mutex_init (&Videoserver::mutex, NULL);

		if(!Videoserver::video){
			Lock lock(mutex);
			if(!Videoserver::video){
				Videoserver::video=new Videoserver();
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
	double updateGameState(GameStatePtr gameState) const;


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

	~Videoserver(){
		delete video;
	};
	friend void update(int);
//czas co jaki videoserwer pobiera inf z symulatora
	static const int updateDeltaTime=100000; //100[ms]
private:
	virtual void execute(void*) ;
	/*@brief pobiera z symulatora pozycje wszytskich robotow na planszy oraz pilki
	 *
	 */
	void update();
	GameStatePtr gameState;
	static Videoserver * video;
	Videoserver();
	Videoserver(const Videoserver& v);
	///czas symulatora podczas wykonania ostatniej aktualizacji
	double lastUpdateTime; //[s]
	//odstep pomiedzy ostatnimi aktualizacjami
	double updateT;//[s]
	//mutex chroniacy dostep do singletonu
	static pthread_mutex_t  mutex;
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

};


#endif /*VIDEOSERVER_H_*/
