/*
 * signal.h
 *
 *  Created on: 2010-04-05
 *      Author: maciek
 */

#ifndef SIGNAL_H_
#define SIGNAL_H_

#include <csignal>     // funkcje: sigaction()
#include <cstdio>
#include <cstdlib>
#include <strings.h>

void sys_signalHandler(int sigType, siginfo_t * info, void * contex);
int sys_catch_signals();

#endif /* SIGNAL_H_ */
