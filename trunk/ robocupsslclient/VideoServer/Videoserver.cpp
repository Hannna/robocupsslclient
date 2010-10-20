#include "Videoserver.h"
#include "../Config/Config.h"
#include <signal.h>
pthread_mutex_t Videoserver::mutex;
struct timeval Videoserver::startTime;

Videoserver * Videoserver::video;
Videoserver::Videoserver()
{
	pthread_mutex_init (&Videoserver::mutex, NULL);
	Videoserver::gameState=GameStatePtr(new GameState());
#ifdef GAZEBO
	lastUpdateTime = 0;//SimControl::getInstance().getSimTime();
#endif
}

double Videoserver::updateGameState(GameStatePtr gameState_) const{
	double currTime;
	pthread_mutex_lock (&Videoserver::mutex);
	if( gameState_.get()!=NULL)
		if( Videoserver::gameState.get()!=NULL )
			(*gameState_)=(*Videoserver::gameState);
	currTime=this->lastUpdateTime;
	pthread_mutex_unlock (&Videoserver::mutex);
	return currTime;
}

/*
double Videoserver::updateGameState(GameStatePtr gameState) {

	struct timeval diff;
	pthread_mutex_lock (&Videoserver::mutex);
	diff=measureTime(stop,&Videoserver::startTime);
	double currTime;
	if(diff.tv_usec>10000){
	#ifdef GAZEBO
		currTime = SimControl::getInstance().getSimTime();
	#endif
		gameState->updateSimTime(currTime);
		//okres czasu od ostatniego update
		updateT = currTime - this->lastUpdateTime;
		this->lastUpdateTime = currTime;

		std::map<std::string,Pose > positions;
	#ifdef GAZEBO
		SimControl::getInstance().getAllPos(positions);
	#endif
		PositionsIt ii=positions.begin();
		std::string model_name;
		for(;ii!=positions.end();++ii){
			model_name=(*ii).first;

			if(model_name.compare("ball")==0){
				gameState->updateBallData(Vector2D((*ii).second.get<0>(),(*ii).second.get<1>()),Vector2D(0.0,0.0));
			}
			else{
				gazebo::PositionIface * posIface=this->posIfaces[model_name];
				double vx,vy,w;
				if(posIface){
					posIface->Lock(1);
					vx = posIface->data->velocity.pos.x;
					vy = posIface->data->velocity.pos.y;
					w = posIface->data->cmdVelocity.yaw;
					posIface->Unlock();
				}
				gameState->updateRobotData(model_name,(*ii).second,Vector2D(vx,vy),w);
			}
		}
		diff=measureTime(start,&Videoserver::startTime);
	}

	pthread_mutex_unlock (&Videoserver::mutex);
	return currTime;
}*/
void Videoserver::update(){
	pthread_mutex_lock (&Videoserver::mutex);
	double currSimTime = SimControl::getInstance().getSimTime();
	this->updateT=currSimTime-this->lastUpdateTime;
	this->lastUpdateTime=currSimTime;

	std::map<std::string,Pose > positions;
#ifdef GAZEBO
	SimControl::getInstance().getAllPos(positions);
#endif
	PositionsIt ii=positions.begin();
	std::string model_name;
	for(;ii!=positions.end();++ii){
		model_name=(*ii).first;

		if(model_name.compare("ball")==0){
			Videoserver::gameState->updateBallData(Vector2D((*ii).second.get<0>(),(*ii).second.get<1>()),Vector2D(0.0,0.0));
		}
		else{
			gazebo::PositionIface * posIface=this->posIfaces[model_name];
			double vx,vy,w;
			if(posIface){
				posIface->Lock(1);
				vx = posIface->data->velocity.pos.x;
				vy = posIface->data->velocity.pos.y;
				w = posIface->data->cmdVelocity.yaw;
				//std::cout<<"vx "<<vx<<"vy "<<vy<<std::endl;
				posIface->Unlock();
			}
			Videoserver::gameState->updateRobotData(model_name,(*ii).second,Vector2D(vx,vy),w);
		}
	}

	pthread_mutex_unlock (&Videoserver::mutex);
	return ;
}
double Videoserver::getUpdateDeltaTime()const{
	return 	updateT;
}

void Videoserver::registerRobot( gazebo::PositionIface *posIface,std::string robotName){
	this->posIfaces[robotName]=posIface;
}

void * runVideoserver(void *){
	useconds_t useconds=10; //10 ms
	sigset_t set;
	sigemptyset (&set);
	sigaddset(&set,SIGALRM);
	pthread_sigmask(SIG_UNBLOCK, &set,NULL);
	struct sigaction act;
	act.sa_handler=update;
	sigaction(SIGALRM, &act,NULL);
//	Videoserver::getInstance().update();
	ualarm(useconds, 0);

	while(true){
		usleep(useconds);
	}


	return 0;
}
void update(int){
	//std::cout<<"dupa"<<std::endl;
	Videoserver::getInstance().update();
	ualarm(1000, 0);//1 ms
}

/*
void Videoserver::InitPrint(std::ofstream & file,std::string fileName,std::string robotName)
{
	double s = 100.0; //skala
	file.open(fileName.c_str(),std::ios::app);
	file.setf(std::ios::fixed);
	file.precision(3);

	file<<"\\begin{picture}"<<Vector2D(Field::EFFECTIVE_SIZE)*s<<"\n";
	//rysowanie boiska
	file<<"\\thicklines\n";
	file<<"\\put(0,0){\\line(0,1){"<<Field::EFFECTIVE_SIZE.y*s<<"}}\n";
	file<<"\\put(0,0){\\line(1,0){"<<Field::EFFECTIVE_SIZE.x*s<<"}}\n";
	file<<"\\put(0,"<<Field::EFFECTIVE_SIZE.y*s<<"){\\line(1,0){"<<Field::EFFECTIVE_SIZE.x*s<<"}}\n";
	file<<"\\put("<< Field::EFFECTIVE_SIZE.x*s <<",0){\\line(0,1){"<<Field::EFFECTIVE_SIZE.y*s<<"}}\n\n";
	file<<"\\thinlines\n";

	file<<"\\put"<< Vector2D(Field::BOTTOM_LEFT)*s <<"{\\line(1,0){"<<Field::AREA_SIZE.x*s<<"}}\n";
	file<<"\\put"<< Vector2D(Field::BOTTOM_LEFT)*s <<"{\\line(0,1){"<<Field::AREA_SIZE.y*s<<"}}\n";
	file<<"\\put"<< Vector2D(Field::TOP_RIGHT)*s <<"{\\line(-1,0){"<<Field::AREA_SIZE.x*s<<"}}\n";
	file<<"\\put"<< Vector2D(Field::TOP_RIGHT)*s <<"{\\line(0,-1){"<<Field::AREA_SIZE.y*s<<"}}\n";
	file<<"\\put"<< Vector2D(Field::CENTER)*s <<"{\\line(1,0){"<<Field::AREA_SIZE.x*s/2.0<<"}}\n";
	file<<"\\put"<< Vector2D(Field::CENTER)*s <<"{\\line(-1,0){"<<Field::AREA_SIZE.x*s/2.0<<"}}\n";
	file<<"\\put"<< Vector2D(Field::CENTER)*s <<"{\\line(0,1){2}}\n";
	file<<"\\put"<< Vector2D(Field::CENTER)*s <<"{\\line(0,-1){2}}\n\n";

}
void Videoserver::Print(std::ofstream& file,std::string robotName)
{
	double s = 100.0; //skala
	//pilka

		Vector2D ball_pos = Videoserver::data.getPosition("ball") * s;
		file<<"\\color{orange}\n";
		file<<"\\put"<<ball_pos<<"{\\circle*{"<< 0.04 * s<< "}}\n";
		file<<"\\color{black}\n\n";

	//robot poruszajacy sie
	{
	Vector2D robot_pos = Videoserver::data.getPosition(robotName) * s;
	file<<"\\color{blue}\n";
	file<<"\\put"<<robot_pos<<"{\\circle*{1.2}}\n";
	file<<"\\color{black}\n\n";
	}


	file<<"%Przeszkody:\n";
	strvec names = Names::getNames();
	strvec::iterator ii;

	for(ii = names.begin(); ii!=names.end(); ii++){
		std::string name = *ii;
		if(name.compare("ball")!=0 && name!=robotName){
			//pozycja przeszkody w ukladzie z plansza
			Vector2D tmp=Videoserver::data.getPosition(name) * s;;
			file<<"\\put"<<tmp<<"{\\circle*{1.2}}\n";

		}
	}
}
void Videoserver::FiniPrint(std::ofstream& file,std::string robotName)
{
	double s = 100.0; //skala
	//pilka

		Vector2D ball_pos = Videoserver::data.getPosition("ball") * s;
		file<<"\\color{orange}\n";
		file<<"\\put"<<ball_pos<<"{\\circle*{"<< 0.04 * s<< "}}\n";
		file<<"\\color{black}\n\n";

		/
	//robot poruszajacy sie
	{
	Vector2D robot_pos = Videoserver::data.getPosition(robotName) * s;
	file<<"\\color{blue}\n";
	file<<"\\put"<<robot_pos<<"{\\circle*{5}}\n";
	//The letter k stands for the following value: 0.5522847498 r
	double r=CVM::L*s;
	double k=0.5522847498*r;
	double x=robot_pos.x;
	double y=robot_pos.y;
	file<<"\\cbezier("<<x-r<<","<<y<<")("<<x-r<<","<<y+k<<")("<<x-k<<","<<y+r<<")("<<x<<","<<y+r<<")\n";
	file<<"\\cbezier("<<x<<","<<y+r<<")("<<x+k<<","<<y+r<<")("<<x+r<<","<<y+k<<")("<<x+r<<","<<y<<")\n";
	file<<"\\cbezier("<<x+r<<","<<y<<")("<<x+r<<","<<y-k<<")("<<x+k<<","<<y-r<<")("<<x<<","<<y-r<<")\n";
	file<<"\\cbezier("<<x<<","<<y-r<<")("<<x-k<<","<<y-r<<")("<<x-r<<","<<y-k<<")("<<x-r<<","<<y<<")\n";
	//file<<"\\put"<<robot_pos<<"{\\circle{"<<2.0*s*CVM::L<<"}}\n";
	//file<<"\\thicklines\n";
	//file<<"\\put"<<robot_pos<<"{\\vector(0,1){30}}\n";
	file<<"\\color{black}\n\n";
	}

**
	file<<"%Przeszkody:\n";
	strvec names = Names::getNames();
	strvec::iterator ii;

	for(ii = names.begin(); ii!=names.end(); ii++){
		std::string name = *ii;
		if(name.compare("ball")!=0 && name!=robotName ){
			//pozycja przeszkody w ukladzie z plansza
			//if(Videoserver::data.getSpeed(name).second.x!=0 ||
			//		Videoserver::data.getSpeed(name).second.y!=0){
			Vector2D tmp=Videoserver::data.getPosition(name) * s;;
			file<<"\\put"<<tmp<<"{\\circle*{5}}\n";
			//}

		}
	}
	file<<"\\end{picture}\n"<<std::endl;
	file.close();
}
*/
