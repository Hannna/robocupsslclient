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

log4cxx::LoggerPtr getLoggerPtr (const char *const name){
	return  log4cxx::Logger::getLogger(std::string(name));
	//return log4cxx::LogManager::getLoggerRepository()->getLogger(std::string(name));
}

