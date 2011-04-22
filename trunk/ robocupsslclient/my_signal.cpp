#include "my_signal.h"

void sys_signalHandler(int sigType, siginfo_t * info, void * contex)
{/*
	char log[MAX_LINE_CHAR]="";
	switch(sigType)
	{

		case SIGHUP:
			LogToFile(SM_ID,"SYS: I've got: SIGHUP",INFO);
		break;
		case SIGINT:
			LogToFile(SM_ID,"SYS: I've got: SIGINT",INFO);
		break;
		case SIGQUIT:
			LogToFile(SM_ID,"SYS: I've got: SIGQUIT",INFO);
		break;
		case SIGILL:
			LogToFile(SM_ID,"SYS: I've got: SIGILL",INFO);
		break;

		case SIGABRT:
			LogToFile(SM_ID,"SYS: I've got: SIGABRT",INFO);
		break;
		case SIGFPE:
			LogToFile(SM_ID,"SYS: I've got: SIGFPE",INFO);
		break;
		case SIGSEGV:{
			LogToFile(SM_ID,"SYS: I've got: SIGSEGV",INFO);
		}
		break;
		case SIGPIPE:
			LogToFile(SM_ID,"SYS: I've got: SIGPIPE",INFO);
		break;

		case SIGALRM:
			LogToFile(SM_ID,"SYS: I've got: SIGALRM",INFO);
		break;
		case SIGTERM:
			LogToFile(SM_ID,"SYS: I've got: SIGTERM",INFO);
		break;
		case SIGUSR1:
			LogToFile(SM_ID,"SYS: I've got: SIGUSR1",INFO);
		break;
		case SIGUSR2:
			LogToFile(SM_ID,"SYS: I've got: SIGUSR2",INFO);
		break;

		case SIGCHLD:
			LogToFile(SM_ID,"SYS: I've got: SIGCHLD",INFO);
		break;
		case SIGTSTP:
			LogToFile(SM_ID,"SYS: I've got: SIGTSTP",INFO);
		break;
		case SIGTTIN:
			LogToFile(SM_ID,"SYS: I've got: SIGTTIN",INFO);
		break;
		case SIGTTOU:
			LogToFile(SM_ID,"SYS: I've got: SIGTTOU",INFO);
		break;

		default:
			sprintf(log,"SYS: I've got unknown signal %d",sigType);
			LogToFile(SM_ID,log,INFO);
		break;
	}
*/
	exit(0);
}
/**
 * Functions tries to overridden all signals that can be sent by system and that in normal case
 * kill the program.
 * @return 0 if everything was ok, -1 if there was some error
*/
int sys_catch_signals()
{
	//char log[MAX_LINE_CHAR]="";
	sigset_t signalMask;
	//inicjuje maske
	sigemptyset(&signalMask);
//	sigaddset(&signalMask,SIGHUP); // Hangup detected on controlling terminal or death of controlling process
	sigaddset(&signalMask,SIGINT); // Interrupt from keyboard
//	sigaddset(&signalMask,SIGQUIT); // Quit from keyboard
//	sigaddset(&signalMask,SIGILL); // Illegal Instruction

//	sigaddset(&signalMask,SIGABRT); // Abort signal from abort(3)
//	sigaddset(&signalMask,SIGFPE); // Floating point exception
//	sigaddset(&signalMask,SIGSEGV); // Invalid memory reference
//	sigaddset(&signalMask,SIGPIPE); // Broken pipe: write to pipe with no readers

//	sigaddset(&signalMask,SIGALRM); // Timer signal from alarm(2)
//	sigaddset(&signalMask,SIGTERM); // Termination signal
//	sigaddset(&signalMask,SIGUSR1); // User-defined signal 1
//	sigaddset(&signalMask,SIGUSR2); // User-defined signal 2

//	sigaddset(&signalMask,SIGCHLD); // Child stopped or terminated
//	sigaddset(&signalMask,SIGTSTP); // Stop typed at tty
//	sigaddset(&signalMask,SIGTTIN); // tty input for background process
//	sigaddset(&signalMask,SIGTTOU); // tty output for background process

	//odblokowuje obsluge sygnalow
	int status=pthread_sigmask(SIG_UNBLOCK, &signalMask,NULL);
 	if(status<0){
 		return -1;
 	}

 	struct sigaction signalAction,old;
 	bzero (&signalAction, sizeof(signalAction));
 	signalAction.sa_handler=NULL;
	signalAction.sa_sigaction=sys_signalHandler;
 	signalAction.sa_flags=SA_SIGINFO;
 	signalAction.sa_restorer = NULL;

 	signalAction.sa_mask=signalMask;
	sigaction(SIGINT, &signalAction,&old);

	return 1;
}