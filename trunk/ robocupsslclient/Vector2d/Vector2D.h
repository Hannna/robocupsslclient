#ifndef VECTOR2D_H_
#define VECTOR2D_H_

#include <iostream>
#include <boost/assert.hpp>

/**
 * @brief  klasa pomocnicza do przechowywania pary liczb 
 * opisujacych pozycje na plaszczyznie 2d.
 * @author Maciej Gąbka
 * @date 7.06.2008
 */
class Vector2D
{
public:
	/// wspolrzedna x wektora 
	double x;
	/// wspolrzedna y wektora
	double y;
	Vector2D();
	Vector2D(double x, double y);
	Vector2D(const Vector2D & );
	~Vector2D();
	///@brief zwraca długość wektora 
	double length() const;
	///@brief zwraca kat pomiedzy wektorami
	double angleTo(const Vector2D &) const;
	///@brief zwraca znak kąta pomiedzy wektorami
	int signOfangleTo(const Vector2D & to) const;
	///@brief zwraca iloczyn skalarny
	double scalarProduct(const Vector2D &)const;

	///@brief rzutuje wybrany punkt na prosta Ax+By+C=0
	Vector2D projectionOn(double a, double b,double c) const;
	double distance(Vector2D to) const;
	Vector2D & operator=(const Vector2D &);
	Vector2D  operator-(const Vector2D &)const;
	Vector2D operator-(const Vector2D &);
	Vector2D operator+(const Vector2D &) const;
	///@brief Metoda pozwalająca na przeskalowanie wektora (mnożenie przez skalar).
	Vector2D operator*(double ) const;
		
	///@brief Metoda zwracająca wektor bazowy obrócony o kąt angle (w radianach).
	Vector2D rotate(double angle);
	
	///@brief zapisuje wektor do strumienia
	friend std::ostream & operator<<(std::ostream & os, const Vector2D & v );
};

#endif /*VECTOR2D_H_*/
