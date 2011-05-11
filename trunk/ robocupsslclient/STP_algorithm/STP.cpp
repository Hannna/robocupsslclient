/*
 * STP.cpp
 *
 *  Created on: 24-04-2011
 *      Author: maciek
 */
#include "STP.h"

#include "../RefereeClient/RefereeClient.h"
#include "../EvaluationModule/EvaluationModule.h"
#include "../Logger/Logger.h"
#include "../Config/Config.h"
#include "../RefereeClient/RefereeClient.h"
#include "../AbstractTactic/AbstractTactic.h"
#include "../Tactics/ShootTactic.h"
#include "../Play/SimplePlay.h"
#include "../AbstractPlay/AbstractPlay.h"
#include "../RefereeClient/RefereeClient.h"

extern const std::string ifaceName;

/*Ogolny model dzialania algorytmu STP
 *
 */
//1. CaptureSensors()
//2. RunPerception()
//3. UpdateWorldModel()
//4. P ← ExecutePlayEngine()
//5. for each robot i ∈ {1, . . . , N }
//       (Ti , T P aramsi ) ← GetTactic(P, i)
//6.
//      (SSMi , SP aramsi ) ← ExecuteTactic(Ti , T P aramsi )
//7.
//8.     if NewTactic(Ti ) then            Si ← SSMi (0)
//9.       (commandi , Si ) ← ExecuteStateMachine(SSMi , Si , SP aramsi )
//10.       robot commandi ← ExecuteRobotControl(commandi )
//11.       SendCommand(i, robot commandi )
//12.



void run_stp(){

	struct timespec req;
	req.tv_sec = 0;
	req.tv_nsec = 10000000;//10ms;
	struct timespec rem;
	bzero(&rem, sizeof(rem) );

	RefereeClient & referee = RefereeClient::getInstance();
	AbstractPlay* play = new SimplePlay();
	RefereeCommands::Command command;

	play->prepareForStart();

	while(true){
		//1. Get Command from sslbox
		command = referee.getCommand();
		//rozpocznij gre
		if( command == RefereeCommands::start ){
			play->execute();
		}
		// zatrzymaj roboty
		if( command == RefereeCommands::halt ){
			play->halt();
		}
		//zatrzymaj roboty 30cm od pilki
		if( command == RefereeCommands::stopGame )
			play->stop();

		//SimControl::getInstance().restart();
		nanosleep(&req, &rem);
	}

}
