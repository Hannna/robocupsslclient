/*
 * test.cpp
 *
 *  Created on: 2009-12-30
 *      Author: maciek
 */

#include "test.h"
using namespace boost::unit_test;


#include "../Config/Config.h"
/******************************************************************************
 * wartosci zwracane przez Unit Test Framework po zakonczeniu testow:
 * boost::exit_success - wszystkie testy zakonczone pomyslnie
 * boost::exit_test_failure - wykryto niekytyczne bledy (nonfatal errors) lub
 * nie powiodla sie inicjalizacja unit test suite
 * boost::exit_exception_failure - pojawily sie bledy krytyczne lub niezlapane
 * wyjatki
 ******************************************************************************/


//____________________________________________________________________________//


/*
test_suite*
init_unit_test_suite( int, char* [] ) {

	framework::master_test_suite().p_name.value = "Przykladowe test_suite";

	//jesli nie korzystamy z globalnego zestawu testow, mozemy definiowac wlasne
	//za pomoca makra o nastepujacej skladni
	//test_suite* test= BOOST_TEST_SUITE( "nazwa zestawu testow" );
	//wywolujac metode add(tak jak ponizej) mozemy dodawac przypadki testowe
	//oraz nowe zestawy testow, ukladajac je w drzewiasta strukture


    framework::master_test_suite().add( BOOST_TEST_CASE( &free_test_function ), 3 );
	boost::shared_ptr<KlasaTestujaca> instance( new KlasaTestujaca(0) );
	framework::master_test_suite().add( BOOST_CLASS_TEST_CASE( &KlasaTestujaca::class_test_case, instance ) );

    return 0;
}
*/
