#include "Set.h"

#include <boost/assert.hpp>
#include <math.h>

/************************************************************
*			SET                                             *
*************************************************************/
Set::Set(double angmin_,double angmax_,double d_):angmin(angmin_),angmax(angmax_), d(d_)
{
	//BOOST_ASSERT(cmin!=angmax);
	BOOST_ASSERT( (this->angmin <= this->angmax) );
}

std::ostream & operator<<(std::ostream & o, const Set &s)
{
	return o<< "SET ("<<s.angmin<<","<<s.angmax<<") d= "<<s.d<<std::endl;
}

Set::Set(const Set& set)
{
	this->angmax=set.angmax;
	this->angmin=set.angmin;
	this->d=set.d;
}

Set & Set::operator=(const Set & s)
{
	this->angmax=s.angmax;
	this->angmin=s.angmin;
	this->d=s.d;
	return *this;
}

bool Set::areSeparated(Set& set)
{
	if( set.angmax <= this->angmin ||  set.angmin >= this->angmax)
		return true;
	//dla katow przechodzacych przez 0
	if( set.angmax * set.angmin  < 0 &&  this->angmax * this->angmin > 0){
		if( set.angmin <  this->angmin  ||  set.angmin > this->angmin ){
		return true;
		}
	}

	if( set.angmax * set.angmin  < 0 &&  this->angmax * this->angmin > 0){

	}
	//else if ( set.cmin >=this->cmin && set.cmin<=this->angmax )
	//return true;
	//else
		return false;
}

//zwraca true jesli dodawany zbior zbior wskazywany przez this zawiera sie w set
bool Set::isIncluded(Set& set)
{
	//oznaczenia z opracowań c1=this->cmin c2=this->angmax
	if( (set.angmin <= this->angmin) && (this->angmax <= set.angmax) )
		return true;
	else
		return false;
}

///zwraca true jesli zbior wskazywany przez this(zbior z listy) zawiera set
bool Set::include(Set& set)
{
	//oznaczenia z opracowań c1=this->cmin c2=this->angmax
	if( (this->angmin <= set.angmin) && ( set.angmax <= this->angmax ))
		return true;
	else
		return false;
}


//zwraca true jesli zbiory na siebie zachodza
bool Set::partlyInclude(Set& set)
{
	if( (this->angmin > set.angmin) && (this->angmin < set.angmax) )
		return true;
	else if((this->angmax > set.angmin) && (this->angmax < set.angmax) )
		return true;
	else
		return false;
}

//zwraca true jesli zbior jest waski
bool Set::isNotCorrect(){

	if(fabs(this->angmin-this->angmax)<0.01){
		return true;

	}
	return false;
}

Set::~Set()
{

}
