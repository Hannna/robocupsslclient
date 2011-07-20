#include "Videoserver.h"

#include <signal.h>

#include "../Config/Config.h"
#include "../Robot/Robot.h"

pthread_mutex_t Videoserver::mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t Videoserver::update_game_state_cv;

struct timeval Videoserver::startTime;

Videoserver * Videoserver::video;

goalArea Videoserver::redGoal;
goalArea Videoserver::blueGoal;

Vector2D Videoserver::redGoalMidPosition;
Vector2D Videoserver::redGoalLeftCornerPosition;
Vector2D Videoserver::redGoalRightCornerPosition;

Vector2D Videoserver::blueGoalMidPosition;
Vector2D Videoserver::blueGoalLeftCornerPosition;
Vector2D Videoserver::blueGoalRightCornerPosition;


Videoserver::Videoserver():log( getLoggerPtr ("app_debug") )
{
   // pthread_mutex_init (&Videoserver::mutex, NULL);
    pthread_cond_init (&Videoserver::update_game_state_cv, NULL);
    LOG_TRACE(log,"create videoserver");
    this->lastUpdateTime = 0;
    stopFlag = false;
    simError = true;
    redGoal = bottom ;
	redGoalMidPosition = Config::getInstance().field.BOTTOM_GOAL_MID_POSITION;
	std::cout<<" redGoalMidPosition "<<redGoalMidPosition<<std::endl;
	redGoalLeftCornerPosition = Config::getInstance().field.BOTTOM_GOAL_LEFT_CORNER;
	redGoalRightCornerPosition= Config::getInstance().field.BOTTOM_GOAL_RIGHT_CORNER;;

	blueGoal = top;
	blueGoalMidPosition = Config::getInstance().field.TOP_GOAL_MID_POSITION;
	std::cout<<" blueGoalMidPosition "<<blueGoalMidPosition<<std::endl;
	blueGoalLeftCornerPosition = Config::getInstance().field.TOP_GOAL_LEFT_CORNER;
	blueGoalRightCornerPosition = Config::getInstance().field.TOP_GOAL_RIGHT_CORNER;

	Videoserver::gameState=GameStatePtr(new GameState());
#ifdef GAZEBO
	lastUpdateTime = 0;
	updateT = 0;
#endif
    LOG_TRACE(log,"VIDEOSERVER has been created");
}

double Videoserver::updateGameState(GameStatePtr& gameState_) const{
	double currTime=0;
	pthread_mutex_lock (&Videoserver::mutex);
	if(this->simError){
		pthread_cond_broadcast(&Videoserver::update_game_state_cv);
		pthread_mutex_unlock (&Videoserver::mutex);
		std::cout<<"simcontrol error"<<std::endl;
		return -1;
	}

	if( gameState_.get()!=NULL)
		if( Videoserver::gameState.get()!=NULL ){
		    if(gameState_->getSimTime( ) < this->lastUpdateTime){
                (*gameState_)=(*Videoserver::gameState);

                currTime=this->lastUpdateTime;
		    }
		    else{
                pthread_cond_wait(&Videoserver::update_game_state_cv,&Videoserver::mutex);
                (*gameState_)=(*Videoserver::gameState);
                currTime=this->lastUpdateTime;
		    }
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
//	if( fabs( currSimTime - this->lastUpdateTime ) < 0.001 ){
//		simError= true;
//		pthread_mutex_unlock (&Videoserver::mutex);
//		return;
//	}

	this->updateT=currSimTime-this->lastUpdateTime;
	this->lastUpdateTime=currSimTime;

    Videoserver::gameState->setSimTime(this->lastUpdateTime);

	std::map<std::string,Pose > positions;
	//std::cout<<"before getAllPos "<<std::endl;

	try{

		int r;
		while(  ( r = SimControl::getInstance().getAllPos(positions) ) == 0 );
		if(r < 0){
			simError= true;
			pthread_mutex_unlock (&Videoserver::mutex);
			return;
		}

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

						posIface->second->Unlock();
					}
				}
				//Videoserver::gameState->updateRobotData(model_name,(*ii).second,Vector2D(vx,vy),w);
				Videoserver::gameState->updateRobotData( Robot::getRobotID(model_name),(*ii).second,Vector2D(vx,vy),w );
			}
		}
	}
	catch( ... ){
		simError = true;
	}

    pthread_cond_broadcast(&Videoserver::update_game_state_cv);

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
	//ualarm(Videoserver::updateDeltaTime, 0);
}

void Videoserver::stop(){
	this->stopFlag = true;
}

void Videoserver::execute(void * arg){
	LOG_TRACE(log,"Start videoserver");
	simError = false;
	sigset_t set;
	sigemptyset (&set);
	sigaddset(&set,SIGALRM);
	pthread_sigmask(SIG_UNBLOCK, &set,NULL);
	struct sigaction act;
	bzero( &act, sizeof( struct sigaction ) );
	act.sa_handler=updateVideo;
	sigaction(SIGALRM, &act,NULL);
	Videoserver::getInstance().update();

//	ualarm(Videoserver::updateDeltaTime, 0);
	struct timespec req;
	req.tv_sec=0;
	req.tv_nsec=Videoserver::updateDeltaTime*1000;
	struct timespec rem;
	bzero( &rem, sizeof(rem) );

	while( !stopFlag  && !simError ){
		nanosleep(&req,&rem);
		Videoserver::getInstance().update();
	}
	LOG_TRACE(log,"Exit from videoserver thread");
		//usleep(Videoserver::updateDeltaTime);
}


const Vector2D  Videoserver::getRedGoalMidPosition(){
	return Videoserver::redGoalMidPosition;
}

const Vector2D  Videoserver::getRedGoalLeftCornerPosition(){
	return Videoserver::redGoalLeftCornerPosition;
}

const Vector2D  Videoserver::getRedGoalRightCornerPosition(){
	return Videoserver::redGoalRightCornerPosition;
}

const Vector2D  Videoserver::getBlueGoalMidPosition(){
	std::cout<<" getblueGoalMidPosition "<<Videoserver::blueGoalMidPosition<<std::endl;
	return Videoserver::blueGoalMidPosition;
}

const Vector2D  Videoserver::getBlueGoalLeftCornerPosition(){
	return Videoserver::blueGoalLeftCornerPosition;
}

const Vector2D  Videoserver::getBlueGoalRightCornerPosition(){
	return Videoserver::blueGoalRightCornerPosition;
}

/*
void Videoserver::setRedGoalMidPosition( const Vector2D & v ){
	std::cout<<" setRedGoalMidPosition "<<v<<std::endl;
	Videoserver::redGoalMidPosition = v;
}

void Videoserver::setRedGoalLeftCornerPosition( const Vector2D & v ){

	Videoserver::redGoalLeftCornerPosition = v;
}

void Videoserver::setRedGoalRightCornerPosition( const Vector2D & v ){
	Videoserver::redGoalRightCornerPosition = v;
}

void Videoserver::setBlueGoalMidPosition( const Vector2D & v ){
	std::cout<<" setBlueGoalMidPosition "<<v<<std::endl;
	Videoserver::blueGoalMidPosition = v;
}

void Videoserver::setBlueGoalLeftCornerPosition( const Vector2D & v ){
	Videoserver::blueGoalLeftCornerPosition = v;
}

void Videoserver::setBlueGoalRightCornerPosition( const Vector2D & v ){
	Videoserver::blueGoalRightCornerPosition = v;
}
*/
void Videoserver::testVideoserver(){

    Videoserver::getInstance().start(NULL);
	GameStatePtr currGameState(new GameState());

    for(int i=0;i< 100;i++){
        double currSimTime=video->updateGameState(currGameState);
    	LOG_INFO(log,"data from "<<currSimTime<<"[s] sim time"<<(*currGameState) );
        //std::cout<<"data from "<<currSimTime<<"[s] sim time"<<(*currGameState)<<std::endl;
        sleep(1);
    }

}
