#ifndef REFEREECLIENT_H
#define REFEREECLIENT_H
#include "../Thread/Thread.h"
#include <boost/thread/mutex.hpp>

#include <arpa/inet.h>

 typedef struct gameStatePacket{
    char cmd;                      // current referee command
    unsigned char cmd_counter;     // increments each time new command is set
    unsigned char goals_blue;      // current score for blue team
    unsigned char goals_yellow;    // current score for yellow team
    unsigned short time_remaining; // seconds remaining for current game stage (network byte order)
  } GameStatePacket;



class RefereeClient : public Thread
{
    public:
        RefereeClient();
        virtual void Execute(void*);
        virtual ~RefereeClient();
    /*@brief funkcja testujaca do test unit
    * sprawdza polaczenie z refereebox oraz odbiera komendy
    */
        void testConnection();
    protected:
    /*@brief tworzy nowy socket
    * @return -1 jesli wystapil blad deskryptor otwartego gniazda jesli OK
    *
    */
        int createServer();
    /*@brief odczytuje komendy z referee box
    *
    */
        void readFromBox();
    private:

    GameStatePacket gameStatePacket_;
    const static int port = 10001;
    int socketfd;
    struct sockaddr_in addr;
    boost::mutex mutex_;
};

#endif // REFEREECLIENT_H
