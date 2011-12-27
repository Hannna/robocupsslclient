#ifndef ROTATIONMATRIX_H_
#define ROTATIONMATRIX_H_
#include <math.h>
#include <ostream>
#include "../Vector2d/Vector2D.h"
#include "../additional.h"

/**
 * @author Maciej Gąbka
 * 
 * @brief klasa pomocnicza reprezetujaca macierz rotacji w wersji 2D
 * 
 * fi to kat w radianach o jaki obrocone sa miedzy soba osie OY ukladu {0} 
 * oraz {1}.  fi nalezy do [-PI;PI] przy czym fi ujemne oznacza obrot w prawo
 * aby uzyskac  wspolrzedne p0 w {0} ukladzie znajac wspolrzedne p1 w {1} ukladzie nalezy:
 * p0=R*p1 + d  gdzie d to przesuniecie poczatkow ukladow wspolrzednych mierzone wzgledem {0}
 * natomiast procedura odwrotna jest nastepujaca p1=R.inverse()*(p0-d) 
 *  
 */
class Pose;
class RotationMatrix
{
public:
	///konstruktor tworzy macierz obrotu odpowiadajaca katowi fi
	//fi to kat o jaki trzeba obrocic uklad globalny zeby otrzymac lokalny
	RotationMatrix(double fi);
	RotationMatrix(const RotationMatrix& rm);
	RotationMatrix & operator =(const RotationMatrix &); 
	///realizuje operacje odwracania macierzy , zwraca macierz odwrotna do R
	RotationMatrix Inverse() const ;
	/// mnozenie macierzy przez wektor
	Vector2D  operator*(const Vector2D& v);
	Pose  operator*(const Pose& v);
	/// mnozenie macierzy, przydatne przy skladaniu obrotow
	RotationMatrix operator*(const RotationMatrix & rm);
	friend std::ostream & operator<<(std::ostream & os, const RotationMatrix & v );
	virtual ~RotationMatrix();
private:
	double fi;
	double a11,a12,
		   a21,a22;
};

#endif /*ROTATIONMATRIX_H_*/
