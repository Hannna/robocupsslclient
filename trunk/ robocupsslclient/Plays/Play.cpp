#include "Play.h"
#include "../Config/Config.h"
#include "../Robot/Robot.h"
#include "../Role/Role.h"
//extern const std::string ifaceName;

const std::string ifaceName="::position_iface";


Robot* Play::red0;//( new Robot(std::string("red0"),ifaceName) );
Robot* Play::red1;//( new Robot(std::string("red1"),ifaceName) );
Robot* Play::red2;//( new Robot(std::string("red2"),ifaceName) );

Robot* Play::blue0;//( new Robot(std::string("blue0"),ifaceName) );
Robot* Play::blue1;//( new Robot(std::string("blue1"),ifaceName) );
Robot* Play::blue2;//( new Robot(std::string("blue2"),ifaceName) );

/*
Robot* Play::red0( new Robot(std::string("red0"),ifaceName) );
Robot* Play::red1( new Robot(std::string("red1"),ifaceName) );
Robot* Play::red2( new Robot(std::string("red2"),ifaceName) );

Robot* Play::blue0( new Robot(std::string("blue0"),ifaceName) );
Robot* Play::blue1( new Robot(std::string("blue1"),ifaceName) );
Robot* Play::blue2( new Robot(std::string("blue2"),ifaceName) );
*/

Play::Play(std::string teamColor): appConfig( Config::getInstance() ), log( getLoggerPtr ( "app_debug" ) )
{
	this->teamColor = teamColor;

	if( teamColor.compare("red") == 0 ){
		LOG_INFO(log,"Create Play for red team");
		role0 = Role( this->red0 );
		role1 = Role( this->red1 );
		role2 = Role( this->red2 );

	}
	else if( teamColor.compare("blue") == 0 ){
		LOG_INFO(log,"Create Play for blue team");
		role0 = Role( this->blue0 );
		role1 = Role( this->blue1 );
		role2 = Role( this->blue2 );
	}
}

void Play::free(){

	delete Play::red0;
	delete Play::red1;
	delete Play::red2;

	delete Play::blue0;
	delete Play::blue1;
	delete Play::blue2;


	Play::red0 = NULL;
	Play::red1 = NULL;
	Play::red2 = NULL;

	Play::blue0 = NULL;
	Play::blue1 = NULL;
	Play::blue2 = NULL;

}

void Play::init(){
	Play::red0 = ( new Robot(std::string("red0"),ifaceName) );
	Play::red1 = ( new Robot(std::string("red1"),ifaceName) );
	Play::red2 = ( new Robot(std::string("red2"),ifaceName) );

	Play::blue0 = ( new Robot(std::string("blue0"),ifaceName) );
	Play::blue1 = ( new Robot(std::string("blue1"),ifaceName) );
	Play::blue2 = ( new Robot(std::string("blue2"),ifaceName) );
}
void Play::halt(){

	LOG_INFO(log,"HALT PLAY");
	role0.getRobotPtr()->stop();
	role1.getRobotPtr()->stop();
	role2.getRobotPtr()->stop();
	//role3->getRobot()->stop();
}

Play::~Play()
{
    //dtor
}
