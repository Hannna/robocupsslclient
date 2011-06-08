#ifndef REFEREECLIENT_H
#define REFEREECLIENT_H

#include "../Thread/Thread.h"
#include "../queue.h"
#include "../Logger/Logger.h"
#include "../Lock/Lock.h"
#include "../Robot/Robot.h"

#include <queue>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/asio.hpp>


 struct gameStatePacket{
    char cmd;                      // current referee command
    unsigned char cmd_counter;     // increments each time new command is set
    unsigned char goals_blue;      // current score for blue team
    unsigned char goals_yellow;    // current score for yellow team
    unsigned short time_remaining; // seconds remaining for current game stage (network byte order)
  }__attribute__ ((packed));

  typedef struct gameStatePacket GameStatePacket;

  std::ostream& operator<<(std::ostream& os, const GameStatePacket& gsp);



//extern SimpleBlockingQueue<>;

namespace RefereeCommands{
	typedef enum Command_{
		//Control commands
			halt = 1 ,// 	                H
			stopGame = 2,// 	            S
			ready = 3,// 	               ' '
			start = 4,// 	                s
		//Game Notifications
			first_half = 5,// 	        	1
			half_time = 6,// 	        	h
			second_half = 7,// 	        	2
			overtime_half_1 = 8,// 	        o
			overtime_half_2 = 9,// 	        O
			penalty_shootout = 10,// 	    a

		//Command type 				 Yellow Team Command 	Blue Team Command
		//=================================================================================
		//Game restarts
			kick_off = 11,// 	                k                   K
			penalty = 12,// 	            	    p 	                P
			direct_free_kick = 13,// 	        f 	                F
			indirect_free_kick = 14,// 	        i 	                I
		//Extras
			timeout = 15, //	                	t 	                T
			timeout_end = 16,// 	                z 	                z
			goal_scored = 17,// 	                g 	                G
			decrease_goal_score = 18,// 	        d 	                D
			yellow_card = 19,// 	                y 	                Y
			red_card = 20,// 	                r 	                R
			cancel = 21,//						c
			unknown = 22
	}Command;
}

std::ostream& operator<<(std::ostream& os, const RefereeCommands::Command& command);

class RefereeClient : public Thread
{
public:

	static RefereeClient& getInstance(){
		static RefereeClient* refereeClient;

		if(!refereeClient){
			boost::interprocess::scoped_lock<boost::mutex> guard(RefereeClient::mutex_);
			//LockGuard guard(RefereeClient::mutex_);
			if(!refereeClient)
				refereeClient = new RefereeClient();
		}
		return *refereeClient;
	}

	/*@brief zwraca biezacy stan meczu
	*
	*/
     RefereeCommands::Command getCommand()  ;

     Robot::robotID getTeamId(){
    	 return this->teamID;
     }

     void stop(){
    	 this->stopTh = true;
     }
    protected:
        virtual void execute(void*);

    /*@brief odczytuje komendy z referee box
    *
    */
        void readMsgFromBox();

    private:
        //static Mutex mutex_;
        RefereeCommands::Command castToCommand(const char c);

        RefereeClient();
        RefereeClient(const RefereeClient &);
        RefereeClient& operator=(const RefereeClient &);

        std::queue<RefereeCommands::Command> newCommands;

    /*@brief funkcja testujaca do test unit
    * sprawdza polaczenie z refereebox oraz odbiera komendy
    */
        void testConnection();


        virtual ~RefereeClient();


        GameStatePacket gameStatePacket;
        const static int port = 10001;
        static boost::mutex mutex_;
        RefereeCommands::Command cmd;
        Robot::robotID teamID;
        boost::asio::io_service io_service;
        boost::asio::ip::udp::socket socket;
        boost::asio::ip::udp::endpoint local_endpoint;

        const log4cxx::LoggerPtr log;
        //stop referee thread
        bool stopTh;

};



#endif // REFEREECLIENT_H
