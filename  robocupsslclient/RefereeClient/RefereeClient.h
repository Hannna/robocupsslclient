#ifndef REFEREECLIENT_H
#define REFEREECLIENT_H

#include "../Thread/Thread.h"
#include "../queue.h"
#include "../Logger/Logger.h"

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
			halt,// 	                        H
			stopGame,// 	                        S
			ready,// 	                        ' '
			start,// 	                        s
		//Game Notifications
			first_half,// 	        1
			half_time,// 	        h
			second_half,// 	        2
			overtime_half_1,// 	        o
			overtime_half_2,// 	        O
			penalty_shootout,// 	        a

		//Command type 	Command Description 	Yellow Team Command 	Blue Team Command
		//=================================================================================
		//Game restarts
			kick_off,// 	                k                       K
			penalty,// 	                p 	                P
			direct_free_kick,// 	        f 	                F
			indirect_free_kick,// 	        i 	                I
		//Extras
			timeout, //	                t 	                T
			timeout_end,// 	                z 	                z
			goal_scored,// 	                g 	                G
			decrease_goal_score,// 	        d 	                D
			yellow_card,// 	                y 	                Y
			red_card,// 	                r 	                R
			cancel//							c
	}Command;
}
class RefereeClient : public Thread
{
public:



    public:
        RefereeClient();

    /*@brief funkcja testujaca do test unit
    * sprawdza polaczenie z refereebox oraz odbiera komendy
    */
        void testConnection();

	/*@brief zwraca biezacy stan meczu
	*
	*/
        RefereeCommands::Command getCommand();

        virtual ~RefereeClient();
    protected:
        virtual void execute(void*);

    /*@brief odczytuje komendy z referee box
    *
    */
        void readMsgFromBox();

    private:
        RefereeCommands::Command castToCommand(const char c);
    private:

        GameStatePacket gameStatePacket;
        const static int port = 10001;
        boost::mutex mutex_;

        const log4cxx::LoggerPtr log;

};



#endif // REFEREECLIENT_H
