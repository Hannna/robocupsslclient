#include "RotationMatrix.h"

RotationMatrix::RotationMatrix(double fi)
{
	this->fi=fi;
	a11=cos(fi);
	a12=-sin(fi);
	a21=sin(fi);
	a22=cos(fi);
}
RotationMatrix::RotationMatrix(const RotationMatrix& rm)
{
	this->a11=rm.a11;
	this->a12=rm.a12;
	this->a21=rm.a21;
	this->a22=rm.a22;
	this->fi=rm.fi;
}

RotationMatrix & RotationMatrix::operator =(const RotationMatrix & rm)
{
	this->a11=rm.a11;
	this->a12=rm.a12;
	this->a21=rm.a21;
	this->a22=rm.a22;
	this->fi=rm.fi;
	
	return *this;
}
RotationMatrix RotationMatrix::Inverse()const
{
	return RotationMatrix(-fi);
	//a11=cos(fi);
	//a12=sin(fi);
	//a21=-sin(fi);
	//a22=cos(fi);
}
Vector2D  RotationMatrix::operator*(const Vector2D &v)
{
	return Vector2D(a11*v.x +a12*v.y,a21*v.x+a22*v.y);
}
Pose  RotationMatrix::operator*(const Pose& p){
	return Pose(a11*p.get<0>() +a12*p.get<1>(),a21*p.get<0>()+a22*p.get<1>(),p.get<2>() + fi);
}
RotationMatrix RotationMatrix::operator*(const RotationMatrix & rm)
{
	RotationMatrix wynik(0);
	wynik.a11=this->a11*rm.a11 + this->a12*rm.a21;
	wynik.a12=this->a11*rm.a12 + this->a12*rm.a22;
	wynik.a21=this->a21*rm.a11 + this->a22*rm.a21;
	wynik.a22=this->a21*rm.a12 + this->a22*rm.a22;

	return wynik;
}
 std::ostream & operator<<(std::ostream & os, const RotationMatrix & m )
{
	os<<m.a11<<"\t"<<m.a12<<" \n"<<m.a21<<"\t"<<m.a22<<"\n"<<"fi "<<m.fi;
	return os;
}

RotationMatrix::~RotationMatrix()
{
}
