#include "Experiment.h"


std::map<int, Robot*> Experiment::redTeam;
std::map<int, Robot*> Experiment::blueTeam;

Experiment::Experiment(std::fstream & file, WorldDesc wd, bool isDynamic)													
{
	this->wd=wd;
	this->timeLimit=wd.getTimeLimit();
	
	this->robot= boost::shared_ptr<Robot>( new Robot(wd.getFirstRobotName()));

	//this->task=boost::shared_ptr<GoToBallTask>(new GoToBallTask(robot));
	
	std::vector<std::string> redTeam = Config::getInstance().getRedTeam();
	std::vector<std::string>::iterator robotName = redTeam.begin();

	for(int i=0 ;robotName!=redTeam.end();robotName++,i++ ){
		std::cout<< *robotName <<std::endl;
		Play::redTeam[i] = new Robot(*robotName,ifaceName);
	}

	std::vector<std::string> blueTeam = Config::getInstance().getBlueTeam();
	robotName = blueTeam.begin();

	for(int i=0 ;robotName!=blueTeam.end();robotName++, i++ ){
		Play::blueTeam[i] = new Robot(*robotName,ifaceName);
	}


	boost::posix_time::ptime t = boost::posix_time::second_clock::local_time();
	//file<<"Środowisko: "<<wd.getName()<<" "<<to_simple_string(t)<<std::endl;	
	file<<wd.getName()<<"\t"<<to_simple_string(t)<<"\t";
	
	wyjatek = false;
	
	this->init(isDynamic);
}
void Experiment::init(bool isDynamic)
{
	robot->setSpeed(0.0, 0.0);
	SimControl::getInstance().pause();
	
	SimControl::getInstance().moveAwayModels();	//modele są odsuwane, zeby można je było poustawiać bez konfliktów
	
	std::vector<std::string> names = Names::getNames();
	std::vector<std::string>::iterator i;

	for (i=names.begin(); i !=names.end(); i++) {
		std::string name = (*i);
		pos2D pos=wd.getObjPos(name);
		SimControl::getInstance().setSimPos(name.c_str(), pos.first.x, pos.first.y, pos.second);
	}
	
	//jeżeli eksperyment ma być dynamiczny, zadawane są prędkości obiektom
	if (isDynamic){
		if (wd.speeds.size()==0){
			std::cout<<"Blad w "<<wd.getName()<<std::endl;
			std::cout<<"Wybrano eksperyment dynamiczny, a nie okreslono predkosci dla modeli (SPEED: model vl vr)\n";
			exit(0);
		}
		for(std::vector<std::pair<std::string, Vector2D> >::iterator is = wd.speeds.begin(); is!=wd.speeds.end();is++){
			Robot r((*is).first);
			Vector2D speed = (*is).second;
			r.setSpeed(speed.x,speed.y);	//vl, vr
		}
	}
	
	SimControl::getInstance().resume();
	
	this->startTime = SimControl::getInstance().getSimTime();
	  
}
void Experiment::execute()
{
	try{
		task->execute();
	}
	catch(std::string s){
		wyjatek = true;
	}
}
bool Experiment::finished(std::fstream & file)
{
	double timeElapsed = SimControl::getInstance().getSimTime() - startTime;
	//std::cout<<"Time elapsed: "<<timeElapsed<<std::endl;
	
	if (wyjatek){
		boost::posix_time::ptime t = boost::posix_time::second_clock::local_time();
		
		file<<to_simple_string(t)<<"\t";
		file<<0<<"\t";
		file<<this->timeLimit<<std::endl;
		
		std::cout<<"Wystąpiła kolizja! "<<std::endl;
		return true;
	}
	
	if (timeElapsed > this->timeLimit)
	{
	//	file<<"Przekroczono czas ("<<timeLimit<<"):"<<timeElapsed<<"\t";
		boost::posix_time::ptime t = boost::posix_time::second_clock::local_time();
	//	file<<to_simple_string(t)<<std::endl;
		
		file<<to_simple_string(t)<<"\t";
		file<<2<<"\t";
		file<<timeElapsed<<std::endl;
		
		std::cout<<"Przekroczono czas ("<<timeLimit<<"):"<<timeElapsed<<std::endl;
		
		return true;
	}
	
	if (task->finished())
	{
		//file<<"Eksperyment zakonczono powodzeniem w czasie "<<timeElapsed<<"\t";
		boost::posix_time::ptime t = boost::posix_time::second_clock::local_time();
		//file<<to_simple_string(t)<<std::endl;
		
		file<<to_simple_string(t)<<"\t";
		file<<1<<"\t";
		file<<timeElapsed<<std::endl;
		
		std::cout<<"Eksperyment zakonczono powodzeniem w czasie "<<timeElapsed<<std::endl;
		
		return true;
	}
	
	return false;
}
Experiment::~Experiment()
{
	for (std::vector<std::pair<std::string, Vector2D> >::iterator is =
			wd.speeds.begin(); is!=wd.speeds.end(); is++) {
		Robot r((*is).first);
		r.stop(); //vl, vr
	}
}
