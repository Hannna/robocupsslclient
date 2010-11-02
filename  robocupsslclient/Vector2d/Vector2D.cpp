#include "Vector2D.h"
#include <cmath>
Vector2D::Vector2D()
{
	x=0.0;
	y=0.0;
}

Vector2D::Vector2D(double x, double y)
{
	this->x=x;
	this->y=y;
}

Vector2D::~Vector2D()
{
}

Vector2D::Vector2D(const Vector2D & v)
{
	this->x = v.x;
	this->y = v.y;
}

double Vector2D::angleTo(const Vector2D & to) const
{
	double result;
	
	
	result=acos( (to.x*this->x +to.y*this->y)/(to.length()*this->length()) );
	double z=this->x*to.y-this->y*to.x;
	
	
	if(z>=0)
	return result;
	else return -result;
}

int Vector2D::signOfangleTo(const Vector2D & to) const
{
	double z=this->x*to.y-this->y*to.x;	
	if(z>=0)
	return 1;
	else return -1;
}
double Vector2D::scalarProduct(const Vector2D &to)const
{
	double result;
	
	result=acos( (to.x*this->x +to.y*this->y)/(to.length()*this->length()) );
	
	return result;
}

Vector2D Vector2D::projectionOn(double a,double b,double c) const
{
	BOOST_ASSERT(a!=0 || b!=0);
	Vector2D result;
	//prosta typu x=const
	if(b==0 && a!=0){
		result=Vector2D(-c/a,this->y);
	}
	//prosta typu y=const
	else if(a==0 && b!=0){
		result=Vector2D(this->x,-c/b);
	}
	//prosta typu y=ax+b
	else{
		double a2=b/a;
		double b2=this->y-a2*this->x;
		double x=(b2-(-c/b))/((-a/b)-a2);
		double y=a2*x +b2;
		result=Vector2D(x,y);
	}
	return result;
	
}

double Vector2D::length() const
{
	return sqrt( x*x+y*y);	
}

double Vector2D::distance(Vector2D to)const
{
	return sqrt(pow(to.x-this->x,2)+pow(to.y-this->y,2));
}

Vector2D Vector2D::rotate(double angle){
	double d = this->length();

	if( std::isnan(d) ||  d< 0.001)
		return Vector2D(0,0);
	double curr_angle = atan2(y,x);
	angle += curr_angle;
	
	Vector2D res(d * cos(angle), d * sin(angle) );
	return res;
}

Vector2D & Vector2D::operator =(const Vector2D & v)
{
	this->x = v.x;
	this->y = v.y;
	
	return *this;
}



Vector2D  Vector2D::operator-(const Vector2D & v) const
{
	return Vector2D(x-v.x,y-v.y);
}

Vector2D  Vector2D::operator-(const Vector2D & v)
{
	return Vector2D(x-v.x,y-v.y);
}



Vector2D Vector2D::operator +(const Vector2D & v) const
{	
	return Vector2D(x+v.x,y+v.y);
}

Vector2D Vector2D::operator*(double d)const
{
	return Vector2D(x*d,y*d);
}


std::ostream & operator<<(std::ostream & os, const Vector2D & v )
{
	os<<"("<<v.x<<","<<v.y<<")";
	return os;
}


