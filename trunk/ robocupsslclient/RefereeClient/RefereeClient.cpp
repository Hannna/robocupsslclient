#include "RefereeClient.h"

#include <iostream>
#include <boost/interprocess/sync/scoped_lock.hpp>

using boost::asio::ip::udp;

std::ostream& operator<<(std::ostream& os, const GameStatePacket& gsp){
	os<<"command "<<gsp.cmd<<
		" cmd_counter "<<(int)gsp.cmd_counter<<
		" goals_blue "<<(int)gsp.goals_blue<<
		" goals_yellow "<<(int)gsp.goals_yellow<<
		" time_remaining "<<(int)gsp.time_remaining;
	return os;
}
RefereeClient::RefereeClient()
{
	bzero( &gameStatePacket, sizeof(gameStatePacket) );
	gameStatePacket.cmd='H';
	gameStatePacket.cmd_counter=-1;
}

void RefereeClient::execute(void* ){

    readMsgFromBox();
}

/*
 //działajacy kod
 //
int RefereeClient::createServer(){

    //PF_INET oraz AF_INET to jedno i to samo
	socketfd = socket(PF_INET, SOCK_DGRAM, 0);
	if ( socketfd < 0 )
	{
		perror("socket");
		abort();
	}
	addr.sin_family = PF_INET;
	addr.sin_port = htons(10001);
	addr.sin_addr.s_addr = INADDR_ANY;

	if ( bind(socketfd, (struct sockaddr*)&addr, sizeof(addr)) < 0 )
	{
		perror("bind");
		abort();
		return -1;
	}

    return socketfd;
}
*/


/*
int RefereeClient::connectToBox(){
    int portno=10001, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    socketfd = socket(AF_INET,SOCK_DGRAM , IPPROTO_UDP);
    if (socketfd < 0)
    perror("ERROR opening socket");
    server = gethostbyname("169.254.4.142");
    server = gethostbyname("192.168.0.170");

    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    memcpy( server->h_addr, &serv_addr.sin_addr.s_addr, server->h_length );
    serv_addr.sin_port = htons(portno);
    int status;
    if ( (status = connect(socketfd,(struct sockaddr*)(&serv_addr),sizeof(serv_addr) ) ) < 0){
        perror("ERROR connecting");
        return status;
    }
    std::cout<<"connect return "<<status<<std::endl;
}
*/

void RefereeClient::readMsgFromBox(){

	udp::endpoint sender_endpoint;
    GameStatePacket tmpGameStatePacket;

    size_t bytes_read;
    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket(io_service);
    socket.open(udp::v4());

    //gniazdo jest bindowane z INADDR_ANY 0.0.0.0
    udp::endpoint local_endpoint = boost::asio::ip::udp::endpoint( udp::v4(), this->port );

    socket.bind(local_endpoint);

    do{
    	bytes_read = socket.receive_from( boost::asio::buffer(&tmpGameStatePacket,
    			sizeof(tmpGameStatePacket) ), sender_endpoint );

		this->mutex_.lock();
		if(this->gameStatePacket.cmd_counter!=tmpGameStatePacket.cmd_counter){
			std::cout<<tmpGameStatePacket<<std::endl;
		}
		this->gameStatePacket=tmpGameStatePacket;
		this->mutex_.unlock();

    }while ( bytes_read > 0 );
}

/*
void RefereeClient::readFromBox(){

    socklen_t addrsize;
    int bytes_read;
    addrsize=sizeof(addr);
    GameStatePacket tmpGameStatePacket;

    do
	{
		std::cout<<"waiting before recv"<<std::endl;
		bytes_read = recvfrom(socketfd, (void *)&tmpGameStatePacket, sizeof(tmpGameStatePacket), 0, (struct sockaddr*)&addr, &addrsize);
		//bytes_read = recvfrom(sd, buffer, BUFSIZE, 0, (struct sockaddr*)&addr, &addrsize);
		if ( bytes_read > 0 )
		{
		    tmpGameStatePacket.time_remaining=ntohs(tmpGameStatePacket.time_remaining);
		    std::cout<<"cmd= "<<tmpGameStatePacket.cmd<<" cmd counter "<<(int)tmpGameStatePacket.cmd_counter<<
		    " goals blue "<<tmpGameStatePacket.goals_blue<<" goals yellow "<<tmpGameStatePacket.goals_yellow<<
		    " time_remaining "<<tmpGameStatePacket.time_remaining<<std::endl;
		}
		else
			perror("recvfrom");
//TODO uncomment these lines
        this->mutex_.lock();
		this->gameStatePacket_=tmpGameStatePacket;
		this->mutex_.unlock();

	}while ( bytes_read > 0 );
}
*/
void RefereeClient::testConnection(){
    this->start(0);
    getchar();
    return ;
}

RefereeCommands::Command RefereeClient::getCommand(){
	boost::interprocess::scoped_lock<boost::mutex> guard(this->mutex_);
	return castToCommand(this->gameStatePacket.cmd);
}

RefereeCommands::Command RefereeClient::castToCommand(const char c) {
	using namespace RefereeCommands;
	Command cmd;
	switch(c){
		case 'H': cmd=halt;break;
		case 'S': cmd=stop;break;
		case ' ': cmd=ready;break;
		case 's': cmd=RefereeCommands::start;break;
		case '1': cmd=first_half;break;
		case 'h': cmd=half_time;break;
		case '2': cmd=second_half;break;
		case 'o': cmd=overtime_half_1;break;
		case 'O': cmd=overtime_half_2;break;
		case 'a': cmd=penalty_shootout;break;
		case 'k': cmd=kick_off;break;
		case 'p': cmd=penalty;break;
		case 'f': cmd=direct_free_kick;break;
		case 'i': cmd=indirect_free_kick;break;
		case 't': cmd=timeout;break;
		case 'z': cmd=timeout_end;break;
		case 'g': cmd=goal_scored;break;
		case 'd': cmd=decrease_goal_score;break;
		case 'y': cmd=yellow_card;break;
		case 'r': cmd=red_card;break;
		case 'c': cmd=cancel;break;
		default:
			throw "unsupported conversion";
	}
	return cmd;
}
RefereeClient::~RefereeClient()
{

}
