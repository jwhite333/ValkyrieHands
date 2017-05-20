// Class for using pure TCP messages
#ifndef MESSAGE_HANDLER_H
#define MESSAGE_HANDLER_H

#include <poll.h>
#include <stdio.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>
#include <unistd.h>

#define MANAGER 0
#define MAX_CON 10
#define MAX_MSG_SIZE 1024 // bytes
#define LOCALHOST_IPV4 0x7F000001 //127.0.0.1

class messageHandler
{
    public:
        messageHandler();
        int StartTCPServer(int port);
        int PollTCPServer(int timeout = -1);
        int ConnectToTCPServer(const char * ip, int port);
        void SendTCP(int socketFd, void *data, int type, int messageSize);
        void * ReadTCP(int fd);

    private:
        int connectionRequest();
        int _managerFd;
        int _numConnections;
        struct pollfd _pollList[MAX_CON];
        
};

struct TCPData
{
    int managerFd;
    int clientFd[MAX_CON];
    struct pollfd pollList[MAX_CON];
};

#endif