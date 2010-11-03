#include "RefereeClient.h"

#include <errno.h>
#include <stdio.h>
#include <sys/types.h>          /* See NOTES */
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <cstring>
#include <iostream>

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <resolv.h>
#include <arpa/inet.h>

RefereeClient::RefereeClient()
{
    //ctor
}

void RefereeClient::Execute(void* ){

    if(createServer()<0)
        return;
    readFromBox();
}

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
void RefereeClient::readFromBox(){

    socklen_t addrsize;
    int bytes_read;
    addrsize=sizeof(addr);
    GameStatePacket tmpGameStatePacket;

    do
	{
		//std::cout<<"waiting before recv"<<std::endl;
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
        //this->mutex_.lock();
		this->gameStatePacket_=tmpGameStatePacket;
		//this->mutex_.unlock();

	}while ( bytes_read > 0 );
}

void RefereeClient::testConnection(){
    this->Start(0);
    getchar();
    return ;
}
RefereeClient::~RefereeClient()
{
    close(socketfd);
}
