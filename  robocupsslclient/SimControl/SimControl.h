#ifndef SIMCONTROL_H_
#define SIMCONTROL_H_

#ifdef GAZEBO

#include "../additional.h"
#include "../Lock/Lock.h"

#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_io.hpp>
#include <log4cxx/logger.h>

/**
 * @author Maciej Gąbka,  poprawki Kamil Muszyński
 * @date 15.07.2008
 *
 * @brief Klasa odpowiadająca za konfigurowanie  symulatora.
 *
 */
 class SimControl
{
private:
	SimControl();
	SimControl(const SimControl & sc) {};
public:
	///wzorzec singletonu
	 static   SimControl& getInstance(){
		 if(!SimControl::simControl){
		 	LockGuard lock(mutex);
 			if(!SimControl::simControl){
 				//SimControl::simControl=new SimControl();
 				static SimControl sm;
 				SimControl::simControl = &sm;

		 	}
		 }
		 return *SimControl::simControl;
	}
	///restartuje symulator Gazebo
	void restart();
	/// zatrzymuje symulacje
	void pause();
	/// wzawia symulacje
	void resume();
	/// zwraca akutalny czas symulacji
	double getSimTime();
	/// umozliwia polaczenie  PosIface z symulatorem
	#ifdef OLD
        void connectGazeboPosIface(gazebo::PositionIface *posIface,const char* name);
	#else
        void connectGazeboPosIface(libgazebo::PositionIface *posIface,const char* name);
	#endif
	/// ustawia wybrany model na zadanej pozycji
	void setSimPos(const char* name, const Pose &position);
	Vector2D getBallSpeed();
	/// wypelnia x,y,rot odpowiednio do polozenia modelu o nazwie name na planszy
//	void getSimPos(const char* name, double &x,double &y,double  &rot);
	double getModelPos(std::string modelName,Pose &position);
	///powoduje zapisanie przez gazebo pozycji zadanych obiektów do /tmp/gazebo_poses.txt
	//zwraca 0 gdy f-cje trzeba wywolac ponownie
	//zwraca -1 gdy gazebo zostalo odlaczone

	int getAllPos(std::map<std::string,Pose> &positions);
	///do komunikacji z plikiem gazebo_poses.txt, umożliwia zablokowanie zapisu
	void lock();
	///do komunikacji z plikiem gazebo_poses.txt
	void unlock();
	/**
	 * @brief Metoda pozwala na przesunięcie wszystkich modeli poza planszę.
	 *
	 * Pozwala to na późniejsze swobodne ich rozstawienie. Nie dochodzi do sytuacji,
	 * w których modele wpadają na siebie przy rozstawianiu (o ile rozstawienie jest prawidłowe).
	 */
	void moveAwayModels();

	void moveBall(Pose pose);

	void stopBall( );

	///Do debugowania - wypisuje simulationData
	void display();


	virtual ~SimControl();
private:
	static SimControl * simControl;
	static pthread_mutex_t  mutex;
	/// wysyla komende do symulatora np "get_pose"
//	void sendRequestSimIface(const char *name,gazebo::SimulationRequestData::Type type);
	/// oczekuje az symulator odpowie na komende "get_pose"
//	void getResponseSimIface(double &x,double &y,double & rot,const char * name);

	///Funkcja blokująca (do czasu, kiedy gazebo wykona rządanie)
	//bool wait();

	bool wait(int resp);

	 #ifdef OLD
        gazebo::Client* client;
        gazebo::SimulationIface* simIface;
	 #else
     ///wskaznik na klienta odpowiadającego za połączenie z gazebo
	 libgazebo::Client* client;
	 ///instancja interfejsu symulacyjnego
     libgazebo::SimulationIface* simIface;
     #endif
     const log4cxx::LoggerPtr log;
};

#endif

#endif /*SIMCONTROL_H_*/
