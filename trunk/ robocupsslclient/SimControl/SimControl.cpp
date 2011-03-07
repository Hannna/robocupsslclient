#include "SimControl.h"

#ifdef GAZEBO

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "../Logger/Logger.h"
#include "../additional.h"

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
	double result;
	//simIface->Lock(1);
	while(simIface->Lock(1)!=1);
	simIface->data->responseCount=0;
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
	client->Disconnect();
	delete client;
}

void SimControl::getModelPos(std::string model_name_,Pose &position)
{
	//this->wait();
	std::string model_name=std::string("noname::") + model_name_;
	int i=0;
	while(simIface->Lock(1)!=1){
		std::cout<<i++<<std::endl;
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
		if(simIface->data->responseCount > 0){
				double x=0,y=0,rot=0;
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
	simIface->Unlock();
	return;
}

void SimControl::getAllPos(std::map<std::string,Pose > &positions)
{
    //std::cout<<"getAllPos"<<std::endl;
	LOG_DEBUG(log,"start getALLPPos");
	strvec names = Names::getNames();
	strvec::iterator ii;
	//this->wait();
	int i=0;
	while(simIface->Lock(1)!=1){
		std::cout<<i++<<std::endl;
	}
	//simIface->Lock(1);
	simIface->data->responseCount=0;
	for(ii = names.begin(); ii!=names.end(); ii++){
		std::string model_name_ = *ii;
		std::string model_name=std::string("noname::") + model_name_;
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
		if(simIface->data->responseCount > 0){
			for(ii = names.begin(); ii!=names.end(); ii++){
				std::string model_name_ = *ii;
                std::string model_name=std::string("noname::") + model_name_;
				double x=0,y=0,rot=0;
				for(unsigned long i=0;i<simIface->data->responseCount;i++){
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
							 positions[model_name_]=Pose(x,y,rot);
							 std::ostringstream log_msg;
							 log_msg<<"SimControl getAllPos model name "<<model_name_<<" x="<<positions[model_name_].get<0>()<<" y="<<positions[model_name_].get<1>()<<" rot="<<positions[model_name_].get<2>();
                             LOG_DEBUG(log,log_msg.str());

						}
						break;
						}
					default:
						break;
					}
				}
			}
		}
	simIface->data->responseCount=0;
	simIface->Unlock();
	return;
}

void SimControl::lock()
{
	simIface->Lock(1);
}

void SimControl::unlock()
{
	simIface->Unlock();
}

void SimControl::moveAwayModels(){
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

void SimControl::wait(){
	bool ok=false;
	int i=0;
	while(!ok){
		//while(simIface->Lock(1)!=1);
		if(simIface->data->requestCount==0 ){
			ok=true;
			//std::cout<<"simIface->data->"<<simIface->data->responseCount<<std::endl;
		}
		//simIface->Unlock();
		i++;
		usleep(100);
	}
	//std::cout<<i<<std::endl;
	return;
}

#endif
