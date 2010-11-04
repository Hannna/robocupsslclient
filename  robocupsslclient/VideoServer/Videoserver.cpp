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
	lastUpdateTime = 0;
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

double Videoserver::getUpdateDeltaTime()const{
	return 	updateT;
}

void Videoserver::update(){
#ifdef GAZEBO
	pthread_mutex_lock (&Videoserver::mutex);
	double currSimTime = SimControl::getInstance().getSimTime();
	this->updateT=currSimTime-this->lastUpdateTime;
	this->lastUpdateTime=currSimTime;

	std::map<std::string,Pose > positions;
//#ifdef GAZEBO
	SimControl::getInstance().getAllPos(positions);
//#endif
	PositionsIt ii=positions.begin();
	std::string model_name;
	for(;ii!=positions.end();++ii){
		model_name=(*ii).first;

		if(model_name.compare("ball")==0){
			Videoserver::gameState->updateBallData(Vector2D((*ii).second.get<0>(),(*ii).second.get<1>()),Vector2D(0.0,0.0));
		}
		else{
		    #ifdef OLD
		    gazebo::PositionIface * posIface=this->posIfaces[model_name];
		    #else
			libgazebo::PositionIface * posIface=this->posIfaces[model_name];
			#endif
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
#endif
	return ;
}

#ifdef GAZEBO
	#ifdef OLD
	void Videoserver::registerRobot( gazebo::PositionIface *posIface,std::string robotName){
	#else
	void Videoserver::registerRobot( libgazebo::PositionIface *posIface,std::string robotName){
	#endif
		this->posIfaces[robotName]=posIface;
	}
#endif

void updateVideo(int){
	Videoserver::getInstance().update();
	ualarm(Videoserver::updateDeltaTime, 0);
}

void Videoserver::execute(void * arg){
    std::cout<<"Start videoserver"<<std::endl;

	sigset_t set;
	sigemptyset (&set);
	sigaddset(&set,SIGALRM);
	pthread_sigmask(SIG_UNBLOCK, &set,NULL);
	struct sigaction act;
	act.sa_handler=updateVideo;
	sigaction(SIGALRM, &act,NULL);
	ualarm(Videoserver::updateDeltaTime, 0);

	while(true){
		usleep(Videoserver::updateDeltaTime);
	}
}
