#ifndef EXPERIMENT_H_
#define EXPERIMENT_H_

#include <fstream>
#include <iostream>
#include <boost/assert.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "../additional.h"
#include "../Robot/Robot.h"
#include "../WorldDesc/WorldDesc.h"
#include "../SimControl/SimControl.h"
#include "../Task/Task.h"
#include "../Task/GoToBall.h"
#include "../Logger/Logger.h"

//#include "../tasks/GoToBallTask.h"

/**
 * @brief Klasa opisująca pojedynczy eksperyment  z użyciem algorytmu CVM.
 * 
 * Korzysta z niej ExperimentConfig, która odpowiada za wczytanie światów z pliku i 
 * uruchamianie właściwych metod poniższej klasy
 * @author Maciej Gąbka, Kamil Muszyński
 * @date 10.07.2008
 */
class Experiment
{
public:
	/**
	 * Konstruktor eksperymentu
	  @param file - plik, w którym zostaną zapisane wyniki eksperymentu
	  @param wd - obiekt zawierajacy opis swiata, w ktorym rozgrywa sie eksperyment
	  @param isDynamic określa, czy dany eksperyment ma być dynamiczny, czy statyczny		
	 */
	Experiment(std::fstream & file, WorldDesc wd, bool isDynamic);


	/**
	 * @brief Powoduje wykonanie pojedynczego eksperymentu
	 */
	void execute();
	/**
	* @brief Pozwala ocenić, czy eksperyment zakonczyl sie.
	* W momencie zakonczenia dopisuje wyniki do podanego jako argument pliku
	* 
	* @param file Plik, do którego dopisane zostaną wyniki eksperymentu
	* 
	* @return true gdy eksperyment zakończony powodzeniem, oznacza to, ze dane zostaly dopisane do pliku
	* 
	*/
	bool finished(std::fstream & file);

	static void initRobots();
	virtual ~Experiment();
private:
	/**
	 * @brief Ustawia poczatkowe pozycje wszystkich robotów oraz piłki.
	 * 
	 * Wywoływana przez konstruktor automatycznie.
	 * @param isDynamic czy eksperyment jest dynamiczny, co oznacza, ze zostanie podjeta proba
	 * nadania obiektom predkosci zadanych w obiekcie klasy WorldDesc
	 */
	void init(bool isDynamic);
	
	/// opis rozmieszczenia obiektow podczas eksperymentu
	WorldDesc wd;
	///robot będący pod kontrolą eksperymentu
	//boost::shared_ptr<Robot> robot;
	Robot* robot;
	///zadanie przydzielane robotowi
	boost::shared_ptr<Task> task;
	Task::status taskStatus;
	/// czas, w którym robot wystartowal do celu
	double startTime;
	/// limit czasowy trwania eksperymentu
	double timeLimit;
	///flaga informująca o tym, czy robot rzucil wyjatek zwiazany z kolizja z przeszkoda - wtedy 
	///nalezy zakonczyc eksperyment
	bool wyjatek;
	const log4cxx::LoggerPtr log;
    //static std::map<std::string, Robot*> redTeam;
    //static std::map<std::string, Robot*> blueTeam;
	static std::map<std::string, Robot*> robots;
};

#endif /*STATICEXPERIMENT_H_*/
