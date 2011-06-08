#include "SimControl.h"

#ifdef GAZEBO

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "../Logger/Logger.h"
#include "../additional.h"
#include "../Plays/Play.h"

#include "../SimControl/SimControl.h"
#include "../Exceptions/SimulationException.h"

SimControl * SimControl::simControl = NULL;
pthread_mutex_t  SimControl::mutex = PTHREAD_MUTEX_INITIALIZER;

SimControl::SimControl():log(getLoggerPtr("app_debug"))
{

    #ifdef OLD
        this->client = new gazebo::Client();
        this->simIface = new gazebo::SimulationIface();
	#else
        this->client = new libgazebo::Client();
        this->simIface = new libgazebo::SimulationIface();
	#endif
	int serverId=0;
	std::string id("default");
   /// Connect to the libgazebo server
    try
    {
      this->client->ConnectWait(serverId, GZ_CLIENT_ID_USER_FIRST);
    }
    catch (std::string e)
    {
      std::cout << "Gazebo error: Unable to connect to client \n" << e << "\n";
      exit(0);
    }

    /// Open the Simulation Interface
    try
    {
        if(this->simIface!=NULL){
            //this->simIface->Run
            this->simIface->Open(client,"default");
            //ponoc klient ma to zerowac i dekrementowac...
            this->simIface->Lock(1);
            this->simIface->data->responseCount=0;
            this->simIface->data->requestCount=0;
            this->simIface->Unlock();
        }
    }
    catch (std::string e)
    {
      std::cout << "Gazebo error: Unable to connect to the sim interface\n" << e << "\n";
      exit(0);
    }

    this->simIface->Unpause();

}

void SimControl::restart()
{
	LockGuard lock(mutex);
	while(simIface->Lock(1)!=1);
	//this->simIface->Lock(1);
	simIface->data->responseCount=0;
	#ifdef OLD
	gazebo::SimulationRequestData *request = &(simIface->data->requests[simIface->data->requestCount++]);
	request->type =gazebo::SimulationRequestData::RESET;
	#else
    libgazebo::SimulationRequestData *request = &(simIface->data->requests[simIface->data->requestCount++]);
    request->type =libgazebo::SimulationRequestData::RESET;
    #endif

	//simIface->Reset();
    this->simIface->Unlock();
    this->wait();

}
void SimControl::pause()
{
	LockGuard lock(mutex);
	//this->wait();
	while(simIface->Lock(1)!=1);
	//this->simIface->Lock(1);
	simIface->data->responseCount=0;
	if(this->simIface->data->state==1){
		#ifdef OLD
		gazebo::SimulationRequestData *request = &(simIface->data->requests[simIface->data->requestCount++]);
		request->type =gazebo::SimulationRequestData::PAUSE;
		#else
		libgazebo::SimulationRequestData *request = &(simIface->data->requests[simIface->data->requestCount++]);
		request->type =libgazebo::SimulationRequestData::PAUSE;
		#endif
	}
	else{
		std::cout<<"simulation already paused"<<std::endl;
	}
	this->simIface->Unlock();
	this->wait();
}

void SimControl::resume()
{
	LockGuard lock(mutex);
	this->wait();
	while(simIface->Lock(1)!=1);
	//simIface->Lock(1);
	simIface->data->responseCount=0;
	if (simIface->data->state==0){
    #ifdef OLD
    	gazebo::SimulationRequestData *request = &(simIface->data->requests[simIface->data->requestCount++]);
		request->type =gazebo::SimulationRequestData::PAUSE;
    #else
    	libgazebo::SimulationRequestData *request = &(simIface->data->requests[simIface->data->requestCount++]);
		request->type =libgazebo::SimulationRequestData::PAUSE;
    #endif
	}
	else{
		std::cout<<"simulation already run"<<std::endl;
	}
	simIface->Unlock();
	this->wait();
}

double SimControl::getSimTime()
{
	//LockGuard lock(mutex);
	double result;
	simIface->Lock(1);
	result=simIface->data->simTime;
	simIface->Unlock();

	return result;
}
#ifdef OLD
void SimControl::connectGazeboPosIface(gazebo::PositionIface *posIface,const char* name)
#else
void SimControl::connectGazeboPosIface(libgazebo::PositionIface *posIface,const char* name)
#endif

{
     /// Open the Position interface
    try
    {
      posIface->Open(client, name);
    }
    catch (std::string e)
    {
      std::cout << "Gazebo error: Unable to connect to the position interface\n"
      << e << "\n";
      exit(0);
    }
    return;
}
void SimControl::setSimPos(const char* name, Pose &pose)
{
	LockGuard lock(mutex);
	this->wait();
	//std::cout<<"setSimPos name="<<name<<" x="<<x<<" y="<<y<<" rot="<<rot<<std::endl;
		while(simIface->Lock(1)!=1);
		simIface->data->responseCount=0;
		if(simIface->data->requestCount<GAZEBO_SIMULATION_MAX_REQUESTS){
			//std::cout<<"simIface->data->requestCount "<<simIface->data->requestCount<<std::endl;
			#ifdef OLD
			gazebo::SimulationRequestData *request = &(simIface->data->requests[simIface->data->requestCount++]);
			request->type = gazebo::SimulationRequestData::SET_POSE2D;
			#else
			libgazebo::SimulationRequestData *request = &(simIface->data->requests[simIface->data->requestCount++]);
			request->type = libgazebo::SimulationRequestData::SET_POSE2D;
			#endif
			bzero(request->name,512);
			memcpy(request->name, name, strlen(name));
			//sprintf(request->modelName,"%s", name);
			//gazebo::Pose modelPose(gazebo::Vec3(x,y,0.05) , 0, 0, rot);
			//simIface->SetPose3d(name, modelPose );
			request->modelPose.pos.x = pose.get<0>();
			request->modelPose.pos.y = pose.get<1>();
			request->modelPose.pos.z = 0.1;
			//request->modelPose.pos.z = pose<2>.get();

			request->modelPose.yaw = pose.get<2>();
			request->modelPose.roll = 0;
			request->modelPose.pitch = 0;
		}
		simIface->Unlock();
		//waiting until request was executed
		this->wait();
}
SimControl::~SimControl()
{
	//LOG_INFO(log,"try to destroy SimControl");
	//Play::free();
	client->Disconnect();
	delete client;

	this->simIface->Close();
	delete this->simIface;
}

double SimControl::getModelPos(std::string model_name_,Pose &position)
{
	LockGuard lock(mutex);
	//this->wait();
	std::string model_name=std::string("noname::") + model_name_;
	//int i=0;
	libgazebo::Pose pose;

	bool result = false;
	do{
		result = simIface->GetPose2d(model_name.c_str(),pose );
		if(!result)
			std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!simIface->GetPose2d faild"<<std::endl;

	}while(!result);

	//bool result = simIface->GetPose2d(model_name.c_str(),pose );
	//if(!result)
	//	std::cout<<"simIface->GetPose2d faild"<<std::endl;
	position = Pose( pose.pos.x, pose.pos.y, pose.yaw );
	/*
	while(simIface->Lock(1)!=1){
		std::cout<<"simIface->Lock(1) "<< i++<<std::endl;
	}
	//simIface->Lock(1);
	simIface->data->responseCount=0;
		if(simIface->data->requestCount<GAZEBO_SIMULATION_MAX_REQUESTS){
		    #ifdef OLD
            	gazebo::SimulationRequestData *request = &(simIface->data->requests[simIface->data->requestCount++]);
            	request->type = gazebo::SimulationRequestData::GET_POSE2D;
		    #else
            	libgazebo::SimulationRequestData *request = &(simIface->data->requests[simIface->data->requestCount++]);
            	request->type = libgazebo::SimulationRequestData::GET_POSE2D;
			#endif
			bzero(request->name,512);
			memcpy(request->name, model_name.c_str(), strlen(model_name.c_str()));
		}
	simIface->Unlock();
	this->wait();

	//simIface->Lock(1);
	while(simIface->Lock(1)!=1);
	#ifdef OLD
	gazebo::SimulationRequestData *response = NULL;
	#else
	libgazebo::SimulationRequestData *response = NULL;
	#endif
	double simTime = 0;
		if(simIface->data->responseCount > 0){
				double x=0,y=0,rot=0;
				//double degrees = 0;

				for(unsigned long i=0;i<simIface->data->responseCount;i++){
					response = &simIface->data->responses[i];
					switch(response->type){
                    #ifdef OLD
                    case gazebo::SimulationRequestData::GET_POSE2D:{
                    #else
					case libgazebo::SimulationRequestData::GET_POSE2D:{
					#endif
						if(strcmp(simIface->data->responses[i].name,model_name.c_str())==0){
							x= simIface->data->responses[i].modelPose.pos.x; //x
							y= simIface->data->responses[i].modelPose.pos.y; //y
							rot= simIface->data->responses[i].modelPose.yaw; //rot

							//rot = rot * 180.0 / M_PI;

							//rot = ( simIface->data->responses[i].modelPose.yaw * M_PI )/180.0;

							position=Pose(x,y,rot);
							std::ostringstream ois;
							ois<<"SimControl getModelPos model name "<<model_name<<" x="<<x<<" y="<<y
							<<" rot="<<rot<<std::endl;
							//Logger::getInstance().LogToFile(DBG,ois.str().c_str());
							 //std::cout<<model_name<<" x="<<positions[model_name].get<0>()<<" y="<<positions[model_name].get<1>()
							//<<" rot="<<positions[model_name].get<2>()<<std::endl;
						}
						break;
					}
					default:
						break;
					}
				}
		}
	simIface->data->responseCount=0;

	simTime = simIface->data->simTime;
	simIface->Unlock();
	*/
	return this->getSimTime();
}

int SimControl::getAllPos(std::map<std::string,Pose > &positions)
{

	bool result=true;
	LockGuard lock(mutex);
	//wzor pobierania pozycji z symulatora gazebo
/*	 this->Lock(1);
	 this->data->responseCount = 0;
	 SimulationRequestData *request = &(this->data->requests[this->data->requestCount++]);
	 request->type = SimulationRequestData::GET_POSE3D;
	 memset(request->name, 0, 512);
	 strncpy(request->name, name.c_str(), 512);
	 request->name[511] = '\0';
	 this->Unlock();

	 if (!this->WaitForResponse())
		 return false;

	 assert(this->data->responseCount == 1);
	 pose = this->data->responses[0].modelPose;

	 return true;
*/
	LOG_TRACE(log,"start getALLPPos");
	strvec names = Names::getNames();
	strvec::iterator ii;

	//this->wait();

	while(simIface->Lock(1)!=1){
		usleep(100);
	}

	bzero( simIface->data->responses, sizeof( &simIface->data->responses ) );
	bzero( simIface->data->requests, sizeof( &simIface->data->requests ) );

	int requestCount=0;
	simIface->data->requestCount = 0;
	simIface->data->responseCount=0;

	for(ii = names.begin(); ii!=names.end(); ii++){
		std::string model_name_ = *ii;
		std::string model_name=std::string("noname::") + model_name_;
		if( simIface->data->requestCount < GAZEBO_SIMULATION_MAX_REQUESTS ){
			#ifdef OLD
				gazebo::SimulationRequestData *request = &(simIface->data->requests[simIface->data->requestCount++]);
				request->type = gazebo::SimulationRequestData::GET_POSE2D;
			#else
				libgazebo::SimulationRequestData *request = &(simIface->data->requests[simIface->data->requestCount++]);
				request->type = libgazebo::SimulationRequestData::GET_POSE2D;
			#endif
			bzero(request->name,512);
			memcpy(request->name, model_name.c_str(), model_name.length() );
			assert( strlen(request->name)==model_name.length() );
			request->name[511] = '\0';
		}
	}
	requestCount = simIface->data->requestCount;
	simIface->Unlock();

	if( !this->wait(requestCount) ){
		LOG_FATAL(log,"after wait, wait failed");
		std::ostringstream s;
		s<<__FILE__<<":"<<__LINE__;
		//throw SimulationException(s.str());
		//exit(0);
		return -1;
	}

//teraz metoda wait blokuje simIface jesli znajdzie odpowiedz
	//simIface->Lock(1);

	int responseCount=0;
	unsigned int respCount = simIface->data->responseCount;

	#ifdef OLD
		gazebo::SimulationRequestData *response = NULL;
	#else
		libgazebo::SimulationRequestData *response = NULL;
	#endif
		LOG_TRACE(log,"  simIface->data->responseCount "<<simIface->data->responseCount);

		if(simIface->data->responseCount > 0){
			for(ii = names.begin(); ii!=names.end(); ii++){
				std::string model_name_ = *ii;
                std::string model_name=std::string("noname::") + model_name_;
				double x=0,y=0,rot=0;
				for(unsigned long i=0;i < respCount ;i++){
					response = &simIface->data->responses[i];
					switch(response->type){
                    #ifdef OLD
						case gazebo::SimulationRequestData::GET_POSE2D:{
                    #else
						case libgazebo::SimulationRequestData::GET_POSE2D:{
					#endif
						if(strncmp(simIface->data->responses[i].name,model_name.c_str(),model_name.length())==0){
							 x= simIface->data->responses[i].modelPose.pos.x; //x
							 y= simIface->data->responses[i].modelPose.pos.y; //y
							 rot= simIface->data->responses[i].modelPose.yaw; //rot

							 bzero( simIface->data->responses[i].name, 512 );
							 //bzero( &simIface->data->responses[i], sizeof(libgazebo::SimulationRequestData) );
							 //rot = rot * 180.0 / M_PI;
							 //rot = rot * M_PI/180.0;

							 positions[model_name_]=Pose(x,y,rot);
							 std::ostringstream log_msg;
							 log_msg<<"SimControl getAllPos model name "<<model_name_<<" x="<<positions[model_name_].get<0>()<<" y="<<positions[model_name_].get<1>()<<" rot="<<positions[model_name_].get<2>();
                             LOG_TRACE(log,log_msg.str());
                             responseCount++;
						}
						break;
						}
					default:
						break;
					}
				}
			}
		}
		else{
			result=false;
		}

	simIface->data->responseCount=0;
	simIface->Unlock();

	if( responseCount != requestCount ){
		return 0;
	}

	assert(responseCount == requestCount);

	return 1;
}

void SimControl::lock()
{
	simIface->Lock(1);
}

void SimControl::unlock()
{
	simIface->Unlock();
}

void SimControl::moveBall(Pose pose){

	LockGuard lock(mutex);

	LOG_FATAL(log,"@@@@@@@@@@@@@@@@@@move ball to "<<pose);

//	while(simIface->Lock(1)!=1);
//	simIface->data->responseCount=0;
	std::string model_name("noname::ball");

	//void SimulationIface::SetState(const std::string &name, Pose &modelPose,
	//    Vec3 &linearVel, Vec3 &angularVel, Vec3 &linearAccel,
	//    Vec3 &angularAccel )
	libgazebo::Vec3 p(pose.get<0>(),pose.get<1>(), 0.1);
	libgazebo::Pose pose_(p,pose.get<0>(),pose.get<1>(), 0.1 );
	libgazebo::Vec3 v(0,0,0);

	simIface->SetState(model_name, pose_, v,v,v,v);
			//libgazebo::Vec3(0,0,0),libgazebo::Vec3(0,0,0),
			//slibgazebo::Vec3(0,0,0),libgazebo::Vec3(0,0,0));

	/*if(simIface->data->requestCount<GAZEBO_SIMULATION_MAX_REQUESTS){
		#ifdef OLD
		gazebo::SimulationRequestData *request = &(simIface->data->requests[simIface->data->requestCount++]);
		request->type = gazebo::SimulationRequestData::SET_POSE3D;
		#else
		libgazebo::SimulationRequestData *request = &(simIface->data->requests[simIface->data->requestCount++]);
		request->type = libgazebo::SimulationRequestData::SET_POSE3D;
		#endif
		bzero(request->name,512);
		memcpy(request->name, model_name.c_str(), strlen(model_name.c_str()));

		request->modelPose.pos.x = pose.get<0>();
		request->modelPose.pos.y = pose.get<1>();
		request->modelPose.pos.z = 0.1;//pose.get<2>();

		request->modelPose.yaw = 0;
		request->modelPose.roll = 0;
		request->modelPose.pitch = 0;
	}
*/
//	simIface->Unlock();
	//this->wait();
	LOG_FATAL(log,"@@@@@@@@@@@@@@@@@@ EXIT from move ball to ");
}

/*
void SimControl::stopBall( ){
	LockGuard lock(mutex);

	while(simIface->Lock(1)!=1);
	simIface->data->responseCount=0;


	std::string model_name("noname::ball");
	if(simIface->data->requestCount<GAZEBO_SIMULATION_MAX_REQUESTS){
		#ifdef OLD
		gazebo::SimulationRequestData *request = &(simIface->data->requests[simIface->data->requestCount++]);
		request->type = gazebo::SimulationRequestData::SET_POSE3D;
		#else
		simIface->SetState()
		libgazebo::SimulationRequestData *request = &(simIface->data->requests[simIface->data->requestCount++]);
		request->type = libgazebo::SimulationRequestData;
		#endif
		bzero(request->name,512);
		memcpy(request->name, model_name.c_str(), strlen(model_name.c_str()));

		request->modelPose.pos.x = pose.get<0>();
		request->modelPose.pos.y = pose.get<1>();
		request->modelPose.pos.z = 0.1;//pose.get<2>();

		request->modelPose.yaw = 0;
		request->modelPose.roll = 0;
		request->modelPose.pitch = 0;
	}

	simIface->Unlock();
	this->wait();

}
*/
void SimControl::moveAwayModels(){

	LockGuard lock(mutex);
	strvec names = Names::getNames();
	strvec::iterator ii;
	//simIface->Lock(1);
	while(simIface->Lock(1)!=1);
	simIface->data->responseCount=0;
	double y=-2;
	for(ii = names.begin(); ii!=names.end(); ii++){
		std::string model_name = *ii;
		if(simIface->data->requestCount<GAZEBO_SIMULATION_MAX_REQUESTS){
		    #ifdef OLD
			gazebo::SimulationRequestData *request = &(simIface->data->requests[simIface->data->requestCount++]);
			request->type = gazebo::SimulationRequestData::SET_POSE3D;
			#else
			libgazebo::SimulationRequestData *request = &(simIface->data->requests[simIface->data->requestCount++]);
			request->type = libgazebo::SimulationRequestData::SET_POSE3D;
			#endif
			bzero(request->name,512);
			memcpy(request->name, model_name.c_str(), strlen(model_name.c_str()));
			double x = 0.0;
			y-=0.5;
			double rot=0;
			request->modelPose.pos.x = x;
			request->modelPose.pos.y = y;
			request->modelPose.pos.z = 0.05;

			request->modelPose.yaw = rot;
			request->modelPose.roll = 0;
			request->modelPose.pitch = 0;
		}
	}
	simIface->Unlock();
	this->wait();

}


void SimControl::display(){
/*	simIface->Lock(1);
	std::cout<<"Sim control data\n";
	std::cout<<"\tsimTime "<<simIface->data->simTime<<std::endl;
	std::cout<<"\tpauseTime "<<simIface->data->pauseTime<<std::endl;
	std::cout<<"\tstate "<<simIface->data->state<<std::endl;
	std::cout<<"\tpause "<<simIface->data->pause<<std::endl;
	std::cout<<"\treset "<<simIface->data->reset<<std::endl;
	std::cout<<"\tsave "<<simIface->data->save<<std::endl;
	std::cout<<"\tmodel_name "<<simIface->data->model_name<<std::endl;
	std::cout<<"\tmodel_req "<<simIface->data->model_req<<std::endl;
	simIface->Unlock();
	*/
}

// z symulatora z gazebo
/*
bool SimulationIface::WaitForResponse()
{
  // Wait for the response
  double timeout = 3.0;
  struct timeval t0, t1;
  gettimeofday(&t0, NULL);
  struct timespec sleeptime = {0, 1000000};

  while(this->data->responseCount == 0)
  {
    gettimeofday(&t1, NULL);
    if(((t1.tv_sec + t1.tv_usec/1e6) - (t0.tv_sec + t0.tv_usec/1e6))
        > timeout)
    {
      return false;
    }
    nanosleep(&sleeptime, NULL);
  }

  return true;
}
*/

bool SimControl::wait(){
	LOG_FATAL(log,"start waiting ");

	bool ok=false;

	struct timespec req;
	req.tv_sec=0;
	req.tv_nsec=1000000; //1ms
	struct timespec rem;
	bzero( &rem, sizeof(rem) );

	struct timespec  startTime;
	measureTime( start, &startTime );
	double endTime;

	while(!ok){
//		if( simIface->data->requestCount==0 && simIface->data->responseCount > 0){
		if( ( this->simIface->data->requestCount == 0 ) && ( this->simIface->data->responseCount > 0 ) ){
			LOG_TRACE(log," wait simIface->data->responseCount "<<this->simIface->data->responseCount);
			ok=true;
			continue;
		}
		nanosleep(&req,&rem);
		endTime=measureTime( stop, &startTime );
		//3 sek jest ustawiony timeout w symulatorze gazebo
		if(endTime>5000){
			LOG_FATAL(log,"wait for request "<<endTime<<"ms");
			return false;
		}
	}

	return true;
}

bool SimControl::wait(int resp){
	LOG_TRACE(log,"start waiting for "<<resp<<" responses");
	bool ok=false;

	struct timespec req;
	req.tv_sec=0;
	req.tv_nsec=4000000; //4ms
	struct timespec rem;
	bzero( &rem, sizeof(rem) );

	struct timespec  startTime;
	measureTime( start, &startTime );
	double endTime;

	while(!ok){
		this->simIface->Lock(1);
		if( ( this->simIface->data->requestCount == 0 ) || ( this->simIface->data->responseCount == resp ) ){
			LOG_TRACE(log,"return from wait. simIface->data->responseCount "<<this->simIface->data->responseCount);
			ok=true;

			//jesli mamy rezultat to nie odblokowujemy
			//this->simIface->Unlock();
			continue;
		}
		else{
			LOG_TRACE( log,"when waiting for resp simIface->data->responseCount "<<this->simIface->data->responseCount );
			LOG_TRACE( log,"when waiting for resp simIface->data->requestCount "<<this->simIface->data->requestCount );
		}
		this->simIface->Unlock();

		nanosleep(&req,&rem);
		endTime=measureTime( stop, &startTime );
		//3 sek jest ustawiony timeout w symulatorze gazebo
		if(endTime>5000){
			//LOG_FATAL( log," wait simIface->data->requestCount "<<this->simIface->data->requestCount );
			//LOG_FATAL( log," wait simIface->data->responseCount "<<this->simIface->data->responseCount );
			LOG_FATAL( log,"wait for request "<<endTime<<"ms" );
			return false;
		}

	}

	return true;
}

#endif
