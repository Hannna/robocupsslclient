#include "Play.h"
#include "../Config/Config.h"
#include "../Robot/Robot.h"
#include "../Role/Role.h"
//extern const std::string ifaceName;


//Robot* Play::red0;//( new Robot(std::string("red0"),ifaceName) );
//Robot* Play::red1;//( new Robot(std::string("red1"),ifaceName) );
//Robot* Play::red2;//( new Robot(std::string("red2"),ifaceName) );

//Robot* Play::blue0;//( new Robot(std::string("blue0"),ifaceName) );
//Robot* Play::blue1;//( new Robot(std::string("blue1"),ifaceName) );
//Robot* Play::blue2;//( new Robot(std::string("blue2"),ifaceName) );

/*
Robot* Play::red0( new Robot(std::string("red0"),ifaceName) );
Robot* Play::red1( new Robot(std::string("red1"),ifaceName) );
Robot* Play::red2( new Robot(std::string("red2"),ifaceName) );

Robot* Play::blue0( new Robot(std::string("blue0"),ifaceName) );
Robot* Play::blue1( new Robot(std::string("blue1"),ifaceName) );
Robot* Play::blue2( new Robot(std::string("blue2"),ifaceName) );
*/

std::map<int, Robot*> Play::redTeam;
std::map<int, Robot*> Play::blueTeam;

Play::Play(std::string teamColor, const int nrOfROles): appConfig( Config::getInstance() ), log( getLoggerPtr ( "app_debug" ) )
{

	this->teamColor = teamColor;

	if( teamColor.compare("red") == 0 ){
		LOG_INFO(log,"Create Play for red team");
		for(int i = 0; i < nrOfROles;i++ ){
			roles[i] = new Role( this->redTeam[i] );
		}
	}
	else if( teamColor.compare("blue") == 0 ){
		LOG_INFO(log,"Create Play for blue team");
		for(int i = 0; i < nrOfROles;i++ ){
			roles[i] = new Role( this->blueTeam[i] );
		}
	}
}

void Play::free(){

	/*
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
*/
}

void Play::init(){
	std::vector<std::string> redTeam = Config::getInstance().getRedTeam();
	std::vector<std::string>::iterator robotName = redTeam.begin();

	for(int i=0 ;robotName!=redTeam.end();robotName++,i++ ){
		std::cout<<" add robot "<<*robotName<<" to play "<<std::endl;
		Play::redTeam[i] = new Robot( *robotName,Robot::ifaceName );
	}

	/*
	Play::red0 = ( new Robot(std::string("red0"),ifaceName) );
	Play::red1 = ( new Robot(std::string("red1"),ifaceName) );
	Play::red2 = ( new Robot(std::string("red2"),ifaceName) );
	 */
	std::vector<std::string> blueTeam = Config::getInstance().getBlueTeam();
	robotName = blueTeam.begin();

	for(int i=0 ;robotName!=blueTeam.end();robotName++, i++ ){
		std::cout<<" add robot "<<*robotName<<" to play "<<std::endl;
		Play::blueTeam[i] = new Robot(*robotName,Robot::ifaceName);
	}

	/*
	Play::blue0 = ( new Robot(std::string("blue0"),ifaceName) );
	Play::blue1 = ( new Robot(std::string("blue1"),ifaceName) );
	Play::blue2 = ( new Robot(std::string("blue2"),ifaceName) );
	*/
}
void Play::halt(){

	LOG_INFO(log,"HALT PLAY");
	roles[0]->getRobotPtr()->stop();
	roles[1]->getRobotPtr()->stop();
	roles[2]->getRobotPtr()->stop();
	//role3->getRobot()->stop();
}

Play::~Play()
{
    //dtor
}
