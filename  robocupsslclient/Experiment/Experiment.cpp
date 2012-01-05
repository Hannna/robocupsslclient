#include "Experiment.h"
#include "../Config/Config.h"


std::map<std::string, Robot*> Experiment::robots;

//std::map<int, Robot*> Experiment::blueTeam;

Experiment::Experiment(std::fstream & file, WorldDesc wd, bool isDynamic):log( getLoggerPtr ("app_debug") )
{
	this->taskStatus = Task::error;
	this->wd=wd;
	this->timeLimit=wd.getTimeLimit();
	
	//this->robot= boost::shared_ptr<Robot>( new Robot(wd.getFirstRobotName(), Robot::ifaceName));
	this->robot = robots[wd.getFirstRobotName()];

	//czas w ms wszystkich uruchomien rrt w danym tasku
	total_rrt_time = 0;
	//liczba uruchomien rrt w danym tasku
	rrt_iterations = 0;
	max_path_size = 0;
	min_path_size = 0;
	max_tree_size = 0;
	min_tree_size = 0;

	//this->task=boost::shared_ptr<GoToBall>( new GoToBall( robot ) );
	
	/*
	std::vector<std::string> redTeam = Config::getInstance().getRedTeam();
	std::vector<std::string>::iterator robotName = redTeam.begin();

	for( int i=0 ;robotName!=redTeam.end(); robotName++,i++ ){
		std::cout<< *robotName <<std::endl;
		//Experiment::redTeam[i] = new Robot(*robotName,Robot::ifaceName);
		Experiment::robots[*robotName] = new Robot(*robotName,Robot::ifaceName);
	}

	std::vector<std::string> blueTeam = Config::getInstance().getBlueTeam();
	robotName = blueTeam.begin();

	for(int i=0 ;robotName!=blueTeam.end();robotName++, i++ ){
		//Experiment::blueTeam[i] = new Robot(*robotName,Robot::ifaceName);
		Experiment::robots[*robotName] = new Robot(*robotName,Robot::ifaceName);
	}
	*/

	boost::posix_time::ptime t = boost::posix_time::second_clock::local_time();
	//file<<"Środowisko: "<<wd.getName()<<" "<<to_simple_string(t)<<std::endl;	
	file<<wd.getName()<<"\t"<<to_simple_string(t)<<"\t";
	
	wyjatek = false;
	
	this->init(isDynamic);
}

void Experiment::initRobots(){
	std::vector<std::string> redTeam = Config::getInstance().getRedTeam();
	std::vector<std::string>::iterator robotName = redTeam.begin();

	for( int i=0 ;robotName!=redTeam.end(); robotName++,i++ ){
		std::cout<< *robotName <<std::endl;
		//Experiment::redTeam[i] = new Robot(*robotName,Robot::ifaceName);
		Experiment::robots[*robotName] = new Robot(*robotName,Robot::ifaceName);
	}

	std::vector<std::string> blueTeam = Config::getInstance().getBlueTeam();
	robotName = blueTeam.begin();

	for(int i=0 ;robotName!=blueTeam.end();robotName++, i++ ){
		//Experiment::blueTeam[i] = new Robot(*robotName,Robot::ifaceName);
		Experiment::robots[*robotName] = new Robot(*robotName,Robot::ifaceName);
	}

}

void Experiment::init(bool isDynamic)
{
	LOG_INFO(this->log," INIT ");
	robot->stop();
	LOG_INFO(this->log," after curr robot stop ");
	//SimControl::getInstance().pause();
	LOG_INFO(this->log," after pause simulation ");
	SimControl::getInstance().moveAwayModels();	//modele są odsuwane, zeby można je było poustawiać bez konfliktów
	LOG_INFO(this->log," after moveAwayModels ");
	//std::vector<std::string> names = Names::getNames();

	std::vector<std::string> names = Config::getInstance().getRedTeam();
	std::vector<std::string> tmp = Config::getInstance().getBlueTeam();
	names.insert( names.begin(), tmp.begin(), tmp.end() );
	names.push_back( "ball" );

	std::vector<std::string>::iterator i;

	for (i=names.begin(); i !=names.end(); i++) {
		std::string name = (*i);
		Pose pos=wd.getObjPos(name);
		//dodaje nowy margines
		Pose p( pos.get<0>() + Config::getInstance().field.FIELD_MARIGIN, pos.get<1>() + Config::getInstance().field.FIELD_MARIGIN, pos.get<2>()  );
		LOG_INFO(this->log," after set sip pose for  "<<name.c_str() );
		SimControl::getInstance().setSimPos( name.c_str(), p );
	}

	std::vector<std::string> redTeam = Config::getInstance().getRedTeam();
	std::vector<std::string>::iterator robotName = redTeam.begin();
	for( int i=0 ;robotName!=redTeam.end(); robotName++,i++ ){
		//Experiment::redTeam[*robotName]->stop();
		LOG_INFO( this->log," try to stop  "<<*robotName );
		Experiment::robots[*robotName]->stop();
	}

	std::vector<std::string> blueTeam = Config::getInstance().getBlueTeam();
	robotName = blueTeam.begin();
	for(int i=0 ;robotName!=blueTeam.end();robotName++, i++ ){
		//Experiment::blueTeam[*robotName]->stop();
		LOG_INFO( this->log," try to stop  "<<*robotName );
		Experiment::robots[*robotName]->stop();
	}
	
	//jeżeli eksperyment ma być dynamiczny, zadawane są prędkości obiektom
	if (isDynamic){
		if (wd.speeds.size()==0){
			LOG_INFO(this->log," Blad w "<<wd.getName() );
			LOG_INFO(this->log,"Wybrano eksperyment dynamiczny, a nie okreslono predkosci dla modeli (SPEED: model vl vr)");
			exit(0);
		}
		for(std::vector<std::pair<std::string, Vector2D> >::iterator is = wd.speeds.begin(); is!=wd.speeds.end();is++){
			//Robot r( (*is).first, Robot::ifaceName );

			Vector2D speed = (*is).second;
			//obliczanie predkosci liniowej i obrotowej na podstawie predkosci lewego i prawego kola
			double v = ( speed.x+speed.y )/2.0;
			double w = ( speed.y-speed.x )/0.106;
			Vector2D s(0,v);
			Experiment::robots[(*is).first]->setRelativeSpeed( s, w );	//vl, vr
		}
	}
	
	this->task=boost::shared_ptr<GoToPose>( new GoToPose( Videoserver::getInstance().getBallPosition().getPosition(), robot ) );
	this->task->markParam( Task::analyse_all_field );
	if(!Config::getInstance().RRTWayPointsEnabled())
		this->task->wayPointsDisabled();

	//SimControl::getInstance().resume();
	
	this->startTime = SimControl::getInstance().getSimTime();
	  
}
void Experiment::execute()
{
	//try{
		int steps = 1;
		taskStatus = task->execute( NULL,steps );

		total_rrt_time = task->total_rrt_time;
		rrt_iterations = task->rrt_iterations;
		max_path_size = task->max_path_size;
		if( max_path_size > 0 )
			min_path_size = task->min_path_size;

		max_tree_size = task->max_tree_size;
		if( max_tree_size > 0 )
			min_tree_size = task->min_tree_size;

		if( taskStatus == Task::collision ){
			wyjatek = true;
		}
	//}
	//catch(std::string& s){
	//	wyjatek = true;
	//}
}
bool Experiment::finished(std::fstream & file)
{
	double timeElapsed = SimControl::getInstance().getSimTime() - startTime;
	//std::cout<<"Time elapsed: "<<timeElapsed<<std::endl;
	
	if ( wyjatek ){
		boost::posix_time::ptime t = boost::posix_time::second_clock::local_time();
		
		file<<to_simple_string(t)<<"\t";
		file<<0<<"\t";
		//file<<this->timeLimit<<std::endl;
		file<<this->timeLimit<<"\t";
		file<<total_rrt_time<<"\t";
		file<<rrt_iterations<<"\t";
		file<<max_path_size<<"\t";
		file<<min_path_size<<"\t";
		file<<max_tree_size<<"\t";
		file<<min_tree_size<<std::endl;
		file.flush();
		LOG_INFO(this->log,"Wystąpiła kolizja! ");

		return true;
	}
	
	if (timeElapsed > this->timeLimit)
	{
	//	file<<"Przekroczono czas ("<<timeLimit<<"):"<<timeElapsed<<"\t";
		boost::posix_time::ptime t = boost::posix_time::second_clock::local_time();
	//	file<<to_simple_string(t)<<std::endl;
		
		file<<to_simple_string(t)<<"\t";
		file<<2<<"\t";
		//file<<timeElapsed<<std::endl;
		file<<timeElapsed<<"\t";
		file<<total_rrt_time<<"\t";
		file<<rrt_iterations<<"\t";
		file<<max_path_size<<"\t";
		file<<min_path_size<<"\t";
		file<<max_tree_size<<"\t";
		file<<min_tree_size<<std::endl;
		
		LOG_INFO(this->log,"Przekroczono czas ("<<timeLimit<<"):"<<timeElapsed);
		
		return true;
	}
	
	if ( this->taskStatus == Task::ok )
	{
		//file<<"Eksperyment zakonczono powodzeniem w czasie "<<timeElapsed<<"\t";
		boost::posix_time::ptime t = boost::posix_time::second_clock::local_time();
		//file<<to_simple_string(t)<<std::endl;
		
		file<<to_simple_string(t)<<"\t";
		file<<1<<"\t";
		//file<<timeElapsed<<std::endl;
		file<<timeElapsed<<"\t";
		file<<total_rrt_time<<"\t";
		file<<rrt_iterations<<"\t";
		file<<max_path_size<<"\t";
		file<<min_path_size<<"\t";
		file<<max_tree_size<<"\t";
		file<<min_tree_size<<std::endl;
		
		LOG_INFO(this->log," Eksperyment zakonczono powodzeniem w czasie "<<timeElapsed );
		
		return true;
	}
	
	return false;
}
Experiment::~Experiment()
{
	std::map<std::string, Robot*>::iterator ii = Experiment::robots.begin();

	for ( ; ii!=robots.end(); ii++) {
		ii->second->stop();
	}
}
