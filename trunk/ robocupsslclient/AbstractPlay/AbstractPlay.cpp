#include "AbstractPlay.h"
#include "../Config/Config.h"
#include "../Robot/Robot.h"

extern const std::string ifaceName;

AbstractPlay::AbstractPlay(): appConfig( Config::getInstance() ), log( getLoggerPtr ( "app_debug" ) )
{
	red0 = new Robot(std::string("red0"),ifaceName);
	red1 = new Robot(std::string("red1"),ifaceName);
	red2 = new Robot(std::string("red2"),ifaceName);
	blue0 = new Robot(std::string("blue0"),ifaceName);
	blue1 = new Robot(std::string("blue1"),ifaceName);
	blue2 = new Robot(std::string("blue2"),ifaceName);
}


void AbstractPlay::halt(){

	LOG_INFO(log,"HALT PLAY");
	red0->stop();
	red1->stop();
	red2->stop();
	blue0->stop();
	blue1->stop();
	blue2->stop();

}
AbstractPlay::~AbstractPlay()
{
    //dtor
}
