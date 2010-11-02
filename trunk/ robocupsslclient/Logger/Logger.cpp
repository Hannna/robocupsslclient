// C++ Interface: logger
//
// Description:
//
//
// Author: M.Gabka <mgabka@gmail.com>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution

#include "Logger.h"
#include "../Config/Config.h"
int Logger::nrChangesPathFile;
string Logger::det_dir_from_filename(string file_name)
{
	size_t found;
	//cout << "Splitting: " << str << endl;
	found=file_name.find_last_of("/\\");
	//cout << " folder: " << file_name.substr(0,found) << endl;
	//cout << " file: " << file_name.substr(found+1) << endl;

	return file_name.substr(0,found);
}

/**
	true if created
	false not created
*/
short Logger::create_dir_if_doesnt_exist(string dir)
{
	DIR *hdir;
	std::cout<<"dir= "<<dir<<std::endl;
	int found=dir.find_last_of('/');
	dir=dir.substr(0,found);
	std::cout<<"dir= "<<dir<<std::endl;
	hdir=opendir(dir.c_str());
	short ret=0;

	if(hdir==NULL)
	{
		if((ret=mkdir(dir.c_str(),S_IRWXU | S_IRWXG | S_IRWXO))<0)
		{
			//TODO: some error occured
			cerr<<"size of dir="<<dir.size()<<std::endl;
			cerr<<" Oh my god... I couldn't create a folder: "<< dir<<endl;
		}
		//ret=true;
	}
	else closedir(hdir);

	return ret;
}
bool Logger::createFileIfNotExist(string filename){
	struct stat status;
	if( stat(filename.c_str(), &status) <0){
	  ofstream myfile;
	  myfile.open (filename.c_str());
	  myfile << "File doesnt exist. Created by the application\n";
	  myfile.flush();
	  if(myfile.bad()){
		  myfile.close();
		  return false;
	  }
	}
	return true;
}
bool Logger::init_log_dir_and_files(){

	if(create_dir_if_doesnt_exist(this->robotsPath) <0)
		return false;
	if(!createFileIfNotExist(this->robotsPath.c_str()))
		return false;
	if(create_dir_if_doesnt_exist(this->logFile) <0)
		return false;
	if(!createFileIfNotExist(this->logFile.c_str()))
		return false;

	return true;
}
/*
void Logger::changeLogPathFile(std::string fileName){
	log4cxx::LoggerPtr log;
	log=log4cxx::LogManager::getLoggerRepository()->getLogger(std::string("robotsPath"));
	log->removeAllAppenders ();
	//(log->getAppender(this->robotsPath))->close();
	//(create_dir_if_doesnt_exist(fileName));
	log4cxx::LayoutPtr layout(new log4cxx::PatternLayout(std::string("\%m\%n")));
	log4cxx::LoggerPtr robotsPath=log4cxx::Logger::getLogger("robotsPath");
	robotsPath->addAppender(new log4cxx::FileAppender(layout,fileName,true));
	this->robotsPath=fileName;

	//	aptr.close();
}*/
Logger::Logger(){
	this->robotsPath=std::string("/home/maciek/codeblocks/magisterka/bin/Debug/robotsPath.log");
	this->logFile=std::string("/home/maciek/codeblocks/magisterka/bin/Debug/robocup.log");
}

bool Logger::createLoggers(){
	bool result=true;
	try{
		//log4cxx::LayoutPtr layout(new log4cxx::SimpleLayout());
		if(!init_log_dir_and_files())throw std::string("init_log_dir_falied");
		//log4cxx::LayoutPtr layout(new log4cxx::PatternLayout(std::string("\%-5p %d{%d %b %Y %H:%M:%S} [\%t]: \%m\%n")));

		log4cxx::LayoutPtr layout1(new log4cxx::PatternLayout(std::string("\%m\%n")));
		log4cxx::LoggerPtr robotsPath=log4cxx::Logger::getLogger("robotsPath");
		robotsPath->addAppender(new log4cxx::FileAppender(layout1,this->robotsPath,true));

		log4cxx::LayoutPtr layout2(new log4cxx::PatternLayout(std::string("\%-5p %d{dd MMM yyyy HH:mm:ss,SSS} [\%t]: \%m\%n")));
		log4cxx::LoggerPtr logFile=log4cxx::Logger::getLogger("logFile");
		logFile->addAppender(new log4cxx::FileAppender(layout2,this->logFile,true));
	}
	catch(...){
		result=false;
	}
	return result;
}

bool Logger::enableCout(){
	bool result=true;
	try{
		log4cxx::LayoutPtr layout(new log4cxx::SimpleLayout());
		log4cxx::ConsoleAppenderPtr cApp=new log4cxx::ConsoleAppender(layout);
		cApp->setName("console");
		cApp->setTarget(std::string("System.out"));
		log4cxx::Logger::getRootLogger()->addAppender(cApp);
	}
	catch(...){
		result=false;
	}
	return result;
}
bool Logger::disableCout(){
	bool result=true;
	try{
		/// close console appender
		log4cxx::AppenderList list=log4cxx::Logger::getRootLogger()->getAllAppenders();
		log4cxx::AppenderList::iterator ii=list.begin();
		int i=0;
		for(;ii!=list.end();ii++,i++){
			std::string name=(*ii)->getName();
			if(name.compare("console")==0 ){
				(*ii)->close();
				log4cxx::Logger::getRootLogger()->removeAppender(*ii);
			}
		}
	}
	catch(...){
		result=false;
	}
	return result;
}
Logger::~Logger(){
	/// close all appenders
//	log4cxx::AppenderList list=log4cxx::Logger::getRootLogger()->getAllAppenders();
//	log4cxx::AppenderList::iterator ii=list.begin();
//	for(;ii!=list.end();ii++){
//		(*ii)->close();
//	}
//	log4cxx::Logger::getRootLogger()->shutdown();

}

void Logger::LogToFile(enum level level_, std::ostringstream &ois){
	std::string content=ois.str();
	LogToFile(level_,content.c_str());
	ois.str("");
}
void Logger::LogToFile(enum level level_, const char *data,...){
	//#ifdef DEBUG
	if(Config::getInstance().isDebugMode()){
		va_list arg;
		va_start(arg, data);
		log4cxx::LoggerPtr log;
		char buffer[2560];
		vsprintf(buffer,data,arg);
		std::string message(buffer);
	//	setlocale (LC_ALL, "pl_PL");
		setlocale (LC_ALL, "en_UK");

		log=log4cxx::LogManager::getLoggerRepository()->getLogger(std::string("logFile"));

	/*
		switch(level_){
		case PATH:
			log=log4cxx::LogManager::getLoggerRepository()->getLogger(std::string("robotsPath"));
			LOG4CXX_INFO(log, message);
			break;
		default:
			log=log4cxx::LogManager::getLoggerRepository()->getLogger(std::string("robotsPath"));
			break;
		}
		*/
		switch(level_){
		case PATH:
			log=log4cxx::LogManager::getLoggerRepository()->getLogger(std::string("robotsPath"));
			LOG4CXX_INFO(log, message);
			break;
		case INFO:
			LOG4CXX_INFO(log, message);
			break;
		case SYSERR:
			LOG4CXX_ERROR(log, message);
			break;
		case DBG:
			LOG4CXX_DEBUG(log, message);
			break;
		case  FATAL_ERR:
			LOG4CXX_FATAL(log, message);
			break;
		case  WARNING:
			LOG4CXX_WARN(log, message);
			break;
		default: break;
		}
		va_end(arg);
	}
	//#endif

}
