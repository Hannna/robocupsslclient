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
    std::cout<<"create VIDEOSERVER"<<std::endl;
}

double Videoserver::updateGameState(GameStatePtr gameState_) const{
	double currTime=0;
	pthread_mutex_lock (&Videoserver::mutex);

	if( gameState_.get()!=NULL)
		if( Videoserver::gameState.get()!=NULL ){
			(*gameState_)=(*Videoserver::gameState);
			currTime=this->lastUpdateTime;
		}

	pthread_mutex_unlock (&Videoserver::mutex);
	return currTime;
}

double Videoserver::getUpdateDeltaTime()const{
	return 	updateT;
}

void Videoserver::update(){

    //std::cout<<"Videoserver::update() start"<<std::endl;
#ifdef GAZEBO
	pthread_mutex_lock (&Videoserver::mutex);

	//std::cout<<"after lock "<<std::endl;

	double currSimTime = SimControl::getInstance().getSimTime();
	this->updateT=currSimTime-this->lastUpdateTime;
	this->lastUpdateTime=currSimTime;

	std::map<std::string,Pose > positions;
	//std::cout<<"before getAllPos "<<std::endl;

	SimControl::getInstance().getAllPos(positions);

	//std::cout<<"after getAllPos "<<std::endl;

	PositionsIt ii=positions.begin();
	std::string model_name;

	double vx=0,vy=0,w=0;
	PosIfacesIterator posIface;
	for(;ii!=positions.end();++ii){
		model_name=(*ii).first;

		if(model_name.compare("ball")==0){
			Videoserver::gameState->updateBallData(Vector2D((*ii).second.get<0>(),(*ii).second.get<1>()),Vector2D(0.0,0.0));
		}
		else{
			posIface=this->posIfaces.find(model_name);
			if(posIface!=this->posIfaces.end()){
				if(posIface->second!=NULL){
					posIface->second->Lock(1);
					vx = posIface->second->data->velocity.pos.x;
					vy = posIface->second->data->velocity.pos.y;
					//TODO:poprawic pobieranie predkosci katowej robota
					w = posIface->second->data->cmdVelocity.yaw;
					//std::cout<<"vx "<<vx<<"vy "<<vy<<std::endl;
					posIface->second->Unlock();
				}
			}
			Videoserver::gameState->updateRobotData(model_name,(*ii).second,Vector2D(vx,vy),w);
		}
	}

	pthread_mutex_unlock (&Videoserver::mutex);

    //std::cout<<"Videoserver::update() end"<<std::endl;
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
    //std::cout<<"updateVideo"<<std::endl;
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
	Videoserver::getInstance().update();

	ualarm(Videoserver::updateDeltaTime, 0);

	while(true){
		usleep(Videoserver::updateDeltaTime);
	}
}

void Videoserver::testVideoserver(){

    Videoserver::getInstance().start(NULL);
	GameStatePtr currGameState(new GameState());

    for(int i=0;i< 100;i++){
        double currSimTime=video->updateGameState(currGameState);
        std::cout<<"data from "<<currSimTime<<"[s] sim time"<<(*currGameState)<<std::endl;
        sleep(1);
    }

}
