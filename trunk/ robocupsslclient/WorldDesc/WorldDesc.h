#ifndef WORLDDESC_H_
#define WORLDDESC_H_
#include <string>
#include <vector>
#include "boost/tuple/tuple.hpp"
#include "../Vector2d/Vector2D.h"
#include "../additional.h"

typedef std::pair<Vector2D,double> pos2D;
/**
 * @brief Klasa reprezentująca rozmieszczenie robotów oraz pilki na planszy 
 * @author Maciej Gabka
 * @date 14.07.2008
 */
class WorldDesc
{
public:

	WorldDesc();
	/**
	* dodaje pozycje i rotacje obiektu o nazwie name do kolekcji
	*/
	void addObject(double x,double y,double rot,std::string  name);
	/**
	*  zwraca rotacje i pozycje obiektu o zadanej nazwie
	*/
	pos2D getObjPos(std::string name);
	
	
	/// dodaje polecenie nadania prędkości modelowi, wykorzystywane w przypadku 
	/// eksperymentu dynamicznego
	void addSpeedOrder(std::string modelName, double vl, double vr);
	
	virtual ~WorldDesc();
	/**
	*  ustawia nazwe danego rozstawienia robotow
	*/
	void setName(std::string);

	///Zwraca nazwę danego roztawienia robotow
	std::string getName();
	
	///Ustawia maksymalny dopuszczalny czas trwania eksperymentu dla danego rozstawienia
	void setTimeLimit(double t);
	///Zwraca maksymalny dopuszczalny czas trwania eksperymentu dla danego rozstawienia
	double getTimeLimit();
	
	/**
	 * @brief Zwraca ID robota wymienionego na liście obiektów jako pierwszy.
	 * 
	 * Robot ten będzie sterowany w ramach eksperymentu statycznego.
	 */
	std::string getFirstRobotName();
	
	///Do debugowania.
	void display();
	
	/// wektor predkosci do zadania obiektom - nazwa obiektu,(Vl,Vr)
	std::vector<std::pair<std::string, Vector2D> > speeds;
	
private:
	/// kolekcja pozycji poszczegolnych obiektow
	std::vector<boost::tuple<Vector2D,double, std::string>  >objectsPos;
	/// nazwa  opisywanego przez klase stanu na boisku
	std::string name;
	
	/// limit czasu na przejazd w danej sytuacji
	double timeLimit;
	
	
};

#endif /*WORLDDESC_H_*/
