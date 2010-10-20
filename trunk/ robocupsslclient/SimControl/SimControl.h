#ifndef SIMCONTROL_H_
#define SIMCONTROL_H_

#ifdef GAZEBO

#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>

#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_io.hpp>
#include "../additional.h"
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
		static SimControl sc;
		return sc;
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
	void connectGazeboPosIface(gazebo::PositionIface *posIface,const char* name);
	/// ustawia wybrany model na zadanej pozycji
	void setSimPos(const char* name, Pose &position);
	/// wypelnia x,y,rot odpowiednio do polozenia modelu o nazwie name na planszy
//	void getSimPos(const char* name, double &x,double &y,double  &rot);
	void getModelPos(std::string modelName,Pose &positions);
	///powoduje zapisanie przez gazebo pozycji zadanych obiektów do /tmp/gazebo_poses.txt
	void getAllPos(std::map<std::string,Pose> &positions);
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

	///Do debugowania - wypisuje simulationData
	void display();


	virtual ~SimControl();
private:
	/// wysyla komende do symulatora np "get_pose"
//	void sendRequestSimIface(const char *name,gazebo::SimulationRequestData::Type type);
	/// oczekuje az symulator odpowie na komende "get_pose"
//	void getResponseSimIface(double &x,double &y,double & rot,const char * name);

	///Funkcja blokująca (do czasu, kiedy gazebo wykona rządanie)
	void wait();


	///wskaznik na klienta odpowiadającego za połączenie z gazebo
	 gazebo::Client* client;
	 ///instancja interfejsu symulacyjnego
     gazebo::SimulationIface* simIface;
};

#endif

#endif /*SIMCONTROL_H_*/
