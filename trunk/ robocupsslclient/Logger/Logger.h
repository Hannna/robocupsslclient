//
// C++ Interface: logger
//
// Description:
// Author: M.Gabka <mgabka@gmail.com>, (C) 2009
// Copyright: See COPYING file that comes with this distribution


#ifndef LOGGER_H_
#define LOGGER_H_

//#define DEBUG

//enum level {INFO, DBG, SYSERR, FATAL_ERR, WARNING,PATH};

#include <log4cxx/logger.h>
#include <log4cxx/fileappender.h>
#include <log4cxx/consoleappender.h>
#include <log4cxx/simplelayout.h>
#include <log4cxx/patternlayout.h>
#include <log4cxx/logmanager.h>

#include <assert.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include<cstdio>
#include<stdarg.h>

#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>


using namespace std;

log4cxx::LoggerPtr getLoggerPtr (const char * name);
#define LOG_TRACE(l, m) \
{LOG4CXX_TRACE(l, m);} \
//else {LOG4CXX_TRACE(getLogger("log"), m); LOG4CXX_FATAL(getLogger(SM_ID), "loggerPtr was NULL");};

#define LOG_DEBUG(l, m) \
{LOG4CXX_DEBUG(l, m);} \
//else {LOG4CXX_DEBUG(getLogger("log"), m); LOG4CXX_FATAL(getLogger(SM_ID), "loggerPtr was NULL");};

#define LOG_INFO(l, m) \
{LOG4CXX_INFO(l, m);} \
//else {LOG4CXX_INFO(getLogger("log"), m); LOG4CXX_FATAL(getLogger(SM_ID), "loggerPtr was NULL");};

#define LOG_WARN(l, m) \
{LOG4CXX_WARN(l, m);} \
//else {LOG4CXX_WARN(getLogger("log"), m); LOG4CXX_FATAL(getLogger(SM_ID), "loggerPtr was NULL");};

#define LOG_ERROR(l, m) \
{LOG4CXX_ERROR(l, m);} \
//else {LOG4CXX_ERROR(getLogger("log"), m); LOG4CXX_FATAL(getLogger(SM_ID), "loggerPtr was NULL");};

#define LOG_FATAL(l, m) \
{LOG4CXX_FATAL(l, m);} \
//else {LOG4CXX_FATAL(getLogger("log"), m); LOG4CXX_FATAL(getLogger(SM_ID), "loggerPtr was NULL");};

#endif //LOGGER_H_
