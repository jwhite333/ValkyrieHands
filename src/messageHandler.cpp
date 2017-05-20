// Class definition for messageHandler
#include "messageHandler.h"
#include "valkyrieMsg.h"

messageHandler::messageHandler()
{
    _managerFd = -1;
    _numConnections = 0;

    // Create polling object
    for (int i = 0; i < MAX_CON; i++)
    {
        _pollList[i].fd = -1;
        _pollList[i].events = POLLIN;
        _pollList[i].revents = 0;
    }
}

/**
* Start Server:
*   - returns 0 if successful, -1 on error
* 
**/
int messageHandler::StartTCPServer(int port)
{
    // Initialize socket
    int rc;
    struct sockaddr serverAddress;
    struct sockaddr_in *networkAddress = (struct sockaddr_in *)&serverAddress;
    int addressLength = sizeof(struct sockaddr_in);

    // Set attributes
    serverAddress.sa_family = AF_INET;
    networkAddress->sin_addr.s_addr = htons(INADDR_ANY);
    networkAddress->sin_port = htons(port); // 3737 for Valkyrie

    // Bind to 127.0.0.1:3737
    _managerFd = socket(AF_INET, SOCK_STREAM, 0);
    if (_managerFd < 0)
    {
        // Error creating socket
        return -1;
    }
    int option = 1;
    rc = setsockopt(_managerFd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));
    if (rc != 0)
    {
        // Error setting socket options
        return -1;
    }

    rc = bind(_managerFd, &serverAddress, addressLength);
    if (rc < 0)
    {
        // Bind failed, close socket
        do
        {
        } while ((close(_managerFd) == -1) && (errno == EINTR));
        return -1;
    }

    rc = listen(_managerFd, MAX_CON);
    if (rc < 0)
    {
        // Listen failed, close socket
        do
        {
        } while ((close(_managerFd) == -1) && (errno == EINTR));
        return -1;
    }

    // Update pollList
    _pollList[MANAGER].fd = _managerFd;
    _pollList[MANAGER].events = POLLIN;
    _pollList[MANAGER].revents = 0;
    return 0;
}

int messageHandler::PollTCPServer(int timeout)
{
    int rc;
    while (true)
    {
        rc = poll(_pollList, _numConnections + 1, timeout); // Default infinite timeout
        if (rc == 0)
        {
            // Timeout
            return -2;
        }
        else if (rc < 0)
        {
            // Error
            return -1;
        }

        // Read messages from established connections
        for (int i = MANAGER + 1; i < MAX_CON; i++)
        {
            if (_pollList[i].revents != 0)
            {
                // Send fd back to function caller to read
                return _pollList[i].fd;
            }
        }

        // Open new connections through new connection file descriptor
        if (_pollList[MANAGER].revents != 0)
        {
            rc = connectionRequest();
            if (rc < 0)
            {
                // Error 
                return -1;
            }
        }
    }
}

int messageHandler::ConnectToTCPServer(const char * ipAddress, int port)
{
    // Initialize socket
    int rc;
    struct sockaddr serverAddress;
    struct sockaddr_in *networkAddress = (struct sockaddr_in *)&serverAddress;
    int addressLength = sizeof(struct sockaddr_in);

    // Set attributes
    serverAddress.sa_family = AF_INET;
    networkAddress->sin_addr.s_addr = inet_addr(ipAddress);
    networkAddress->sin_port = htons(port);

    // Bind to 127.0.0.1:3737
    int socketFd = socket(AF_INET, SOCK_STREAM, 0);
    if (socketFd < 0)
    {
        // Error opening socket
        return -1;
    }

    int tries = 0;
    while(tries < CONNECT_TRIES)
    {
        rc = connect(socketFd, &serverAddress, addressLength);
        if (rc < 0 && errno == ECONNREFUSED)
        {
            // Port connect is trying to conect to is not available yet, retry
            continue;
        }
        else if (rc < 0)
        {
            // Connect failed, close socket
            std::cout << "Connect errno: " << errno << std::endl;
            do
            {
            } while ((close(socketFd) == -1) && (errno == EINTR));
            return -1;
        }
        else
        {
            // Success
            return socketFd;
        }
    }

    return -1;
}


void messageHandler::SendTCP(int socketFd, void *data, int type, int messageSize)
{
    // Set header fields
    VMSG_HEADER_T * header = (VMSG_HEADER_T *)data;
    header->type = type;
    header->length = messageSize;

    // Send message
    int rc = send(socketFd, data, messageSize, 0); // MSG_DONTWAIT?
    if (rc <= 0)
    {
        // TODO: handle errors
    }

    return;
}

/**
* Read TCP 
*   - must call free() on return pointer
*
**/
messageHandler::Message * messageHandler::ReadTCP(int fd)
{
    Message * returnMsg = (Message *)malloc(sizeof(Message));
    returnMsg->buffer = (char *)malloc(MAX_MSG_SIZE);
    returnMsg->size = recv(fd, returnMsg->buffer, MAX_MSG_SIZE, MSG_DONTWAIT);

    // Error, close socket
    if (returnMsg->size == 0)
    {
        do
        {
        } while ((close(fd) == -1) && (errno == EINTR));

        // Remove fd from pollList by shifting list
        bool removed = false;
        for (int index = 0; index < MAX_CON; index++)
        {
            if (removed == true)
                _pollList[index - 1].fd = _pollList[index].fd;
            if (_pollList[index].fd == fd)
                removed = true;
        }
        _numConnections--;
        FreeMessage(returnMsg);
        return NULL;
    }

    // Nothing read
    else if (returnMsg->size < 0)
    {
        FreeMessage(returnMsg);
        return NULL;
    }

    // Return message for handling
    return returnMsg;
}

int messageHandler::connectionRequest()
{
    int option = 1, rc;
    int connectionFd;
    struct sockaddr_in clientAddress;
    struct linger lngrOption = {0};
    int addressLen = sizeof(struct sockaddr_in);

    connectionFd = accept(_managerFd, (struct sockaddr *)&clientAddress, (unsigned int *)&addressLen);
    if (connectionFd < 0)
    {
        // Accept Failed
        return -1;
    }

    if (_numConnections++ >= MAX_CON)
    {
        // Maximum number of connections reached
        _numConnections--;
        do
        {
        } while ((close(connectionFd) == -1) && (errno == EINTR));
        return 0;
    }

    // turn on keep alives
    rc = setsockopt(connectionFd, SOL_SOCKET, SO_KEEPALIVE, (char *)&option, sizeof(option));
    if (rc != 0)
    {
        // Error setting socket options
        do
        {
        } while ((close(connectionFd) == -1) && (errno == EINTR));
        return -1;
    }

    // turn off linger
    lngrOption.l_onoff = 0;
    rc = setsockopt(connectionFd, SOL_SOCKET, SO_LINGER, (char *)&lngrOption, sizeof(lngrOption));
    if (rc != 0)
    {
        // Error turning off linger
        do
        {
        } while ((close(connectionFd) == -1) && (errno == EINTR));
        return -1;
    }

    // Give new file descriptor to polling struct
    _pollList[_numConnections].fd = connectionFd;
    _pollList[_numConnections].revents = 0;
    return 0;
}

void messageHandler::FreeMessage(Message * message)
{
    free(message->buffer);
    free(message);
    return;
}