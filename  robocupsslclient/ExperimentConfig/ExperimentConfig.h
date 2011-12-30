#ifndef EXPERIMENTCONFIG_H_
#define EXPERIMENTCONFIG_H_

#include <string>
#include <iostream>

//do zapisywania do pliku:
#include <fstream>
#include <ios>

#include <boost/regex.hpp>	//wyrażenia regularne - do interpretacji plików, -lboost_regex!!
#include <boost/lexical_cast.hpp>
#include <sstream>	//do konwersji string->double
#include <math.h>	//M_PI
#include "../WorldDesc/WorldDesc.h"
#include "../VideoServer/Videoserver.h"
#include "../Experiment/Experiment.h"


/**
 * @author Kamil Muszyński
 * @date 14.07.2008
 * 
 * @brief Klasa odpowiadająca za konfigurację procesu eksperymentów.
 * 
 * Eksperymenty opierają się na serii światów, opisanych w specjalnym pliku. 
 * Umożliwia to zdefiniowanie ciągu sytuacji na boisku, 
 * które zostaną poddane eksperymentowi.
 * 
 * @ref skladnia_plik_swiat
 */
class ExperimentConfig
{
public:
	/**
	 * @brief Konstruktor konfiguracji. 
	 * 
	 * @param worldsFile Nazwa pliku zawierającego opis konfiguracji światów użytych w eksperymencie. 
	 * Zobacz także: @ref skladnia_plik_swiat
	 * @param resultsFile Plik, do którego zostaną zapisane rezultaty eksperymentu.
	 * <b>Jeżeli plik istnieje, eksperyment zostanie wznowiony na podstawie stanu pliku.</b>
	 */
	ExperimentConfig(std::string worldsFile, std::string resultsFile);
	~ExperimentConfig();
	

	///@brief Wykonanie serii eksperymentów statycznych.
	void doEx();
	
	///@brief Wykonanie serii eksperymentów dynamicznych
	//void doDynamicEx();
	
	///@brief Do debugowania
	///Wypisuje wczytane światy
	void display();
	
private:
	///Kolekcja wczytanych światów
	std::vector<boost::shared_ptr<WorldDesc> > worlds;
	
	///Plik, do którego zostaną zapisane wyniki eksperymentu.
	std::string resultsFile;
	
	const Videoserver & video;
	const log4cxx::LoggerPtr log;

	///@brief Właściwa metoda uruchamiająca eksperyment, obudowana metodami doStaticEx i doDynamicEx
	///@param isDynamic - jaki rodziaj eksperymentu wykonac - oznacza to nadanie predkosci modelom lub nie	
	void doExperiment( );
	
};

/**
 * @page skladnia_plik_swiat Składnia pliku opisującego świat
 * 
 * @code
 * # komentarz
 * BEGIN nazwa_świata
 * TIME_LIMIT czas(sekundy)
 * nazwa_modelu x y rotacja(stopnie)
 * SPEED: nazwa_modelu_robota predkosc_lewego_kola predkosc_prawego_kola
 * END
 * @endcode
 * 
 * Przykładowo:
 * 
 * @code
 * BEGIN Przykładowa sytuacja
 * #przyjęto limit czasu 2 sekundy
 * TIME_LIMIT 2.0
 * hmt_red0 0.0 2.1 180
 * hmt_red1 0.2 1.1 90
 * #robot red1 będzie miał nadaną prędkość (Vl=0.2,Vr=0.2)
 * SPEED: hmt_red1 0.2 0.2
 * END
 * @endcode
 * 
 * <b>Pierwszy model wymieniony na liście jest tym obiektem, który będzie sterowany 
 * podczas eksperymentu. </b>
 * 
 * MAX_TIME oznacza czas, po upłynięciu którego eksperyment zostanie uznany za nieudany.
 */

#endif /*EXPERIMENTCONFIG_H_*/
