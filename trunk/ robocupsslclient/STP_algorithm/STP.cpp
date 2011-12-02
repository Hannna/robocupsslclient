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
#include "../EvaluationModule/EvaluationModule.h"
#include "../Tactics/Tactic.h"
#include "../Tactics/ShootTactic.h"

#include "../Plays/Play.h"
#include "../Plays/NaivePlay.h"
#include "../Plays/StartPlay.h"

#include "../RefereeClient/RefereeClient.h"

//extern const std::string ifaceName;

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

	log4cxx::LoggerPtr log = getLoggerPtr ("app_debug");

	struct timespec req;
	req.tv_sec = 0;
	req.tv_nsec = 10000000;//10ms;
	struct timespec rem;
	bzero(&rem, sizeof(rem) );

	RefereeClient & referee = RefereeClient::getInstance();
	boost::shared_ptr<Play> redPlay ( new StartPlay("red") );

	boost::shared_ptr<Play> bluePlay ( new StartPlay("blue") );
	RefereeCommands::Command command = RefereeCommands::unknown;
	EvaluationModule::ballState ballState_;

	bluePlay->execute();
	redPlay->execute();

	bluePlay->waitForFinish();
	redPlay->waitForFinish();

	SimControl::getInstance().moveBall( Config::getInstance().field.FIELD_MIDDLE_POSE );

	Robot::robotID teamID = Robot::unknown;

	while( true ){
		try{
			ballState_ = EvaluationModule::getInstance().getBallState( Robot::red0 );
			//1. Get Command from sslbox
			command = referee.getCommand();

			if( command == RefereeCommands::unknown){
				usleep(10000);
				//LOG_INFO( log,"unknown command " );
				continue;
			}
			teamID = referee.getTeamId();

			LOG_INFO( log,"get command "<<command<<" for team "<<teamID );

			//rozpocznij gre
			if( command == RefereeCommands::start ){
				SimControl::getInstance().moveBall( Config::getInstance().field.FIELD_MIDDLE_POSE );

				redPlay = boost::shared_ptr<Play>( new NaivePlay("red") );
				bluePlay = boost::shared_ptr<Play>( new NaivePlay("blue") );
				bluePlay->execute();
				redPlay->execute();
			}
			// zatrzymaj roboty
			else if( command == RefereeCommands::halt ){

				LOG_INFO(log, "HALT " );

				if( bluePlay.get() ){
					LOG_INFO(log, "try to stop blue  team" );
					bluePlay->stop();
				}

				if( redPlay.get() ){
					LOG_INFO(log, "try to stop red  team" );
					redPlay->stop();
				}

				LOG_INFO(log, "before halt" );
				if( bluePlay.get() )
					bluePlay->halt();

				if( redPlay.get() )
					redPlay->halt();

				if( redPlay.get() ){
					redPlay->waitForFinish();
				}

				if( bluePlay.get() ){
					bluePlay->waitForFinish();
				}

				LOG_INFO(log, "disperse finished " );

			//	if( redPlay.get() )
			//		redPlay.reset();

			//	if( bluePlay.get() )
			//		bluePlay.reset();

			}
			//zatrzymaj roboty 30cm od pilki
			else  if( command == RefereeCommands::stopGame ){

				if( bluePlay.get() )
					bluePlay->stop();

				if( redPlay.get() )
					redPlay->stop();

				if( ballState_== EvaluationModule::out ){
					;
				}

				if( redPlay.get() )
					redPlay.reset();

				if( bluePlay.get() )
					bluePlay.reset();


				//delete redPlay.get();
				//delete bluePlay.get();
			}
			else if( command == RefereeCommands::second_half ){
			//	bluePlay->prepareForStart();
			//	redPlay->prepareForStart();
			}
			else if( command == RefereeCommands::goal_scored ){
			//	bluePlay->prepareForStart();
			//	redPlay->prepareForStart();
			}
			else if( command == RefereeCommands::kick_off ){

			//	bluePlay->prepareForStart();
			//	redPlay->prepareForStart();

				bluePlay->waitForFinish();
				redPlay->waitForFinish();

				if( teamID == Robot::red ){
					Vector2D v = EvaluationModule::getInstance().getPositionForThrowIn().getPosition( );
					Pose p(v,0);
					SimControl::getInstance().moveBall( p );
					//redPlay->prepareForKickOff( v );
					//bluePlay->prepareForStart();

					//redPlay->waitForFinish();
					//bluePlay->prepareForStart();

				}
				else{
					Vector2D v = EvaluationModule::getInstance().getPositionForThrowIn().getPosition( );
					Pose p(v,0);
					SimControl::getInstance().moveBall( p );
					//bluePlay->prepareForKickOff( v );
					//redPlay->prepareForStart();

					//bluePlay->waitForFinish();
					//redPlay->waitForFinish();

				}
			}
		}
		catch( ... ){
			LOG_INFO( log, " in main STP loop exception handled " );
			break;
		}

		//SimControl::getInstance().restart();
		nanosleep(&req, &rem); //10ms
	}

}
