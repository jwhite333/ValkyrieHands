// Class Definitions for message logic handling (open and close log file each time to prevent missed data on ctrl+c exit)

#include "logger.h"

void Logger::ThreadEntry(void)
{
    _LogFile.open("RPILog.txt", std::ios::out | std::ios::trunc); // Create a new file each time

    _LogFile << "[" << _name << "] Thread started" << std::endl;

    // Start TCP server
    int rc = StartTCPServer(_serverPort);
    if (rc < 0)
    {
        // Starting server failed
        _LogFile << "[" << _name << "] Starting server failed" << std::endl;
        _LogFile.close();
        ThreadStop();
    }

    _LogFile.close();
    pollCommands();
}

void Logger::pollCommands()
{
    int fd;
    while (_running)
    {
        fd = PollTCPServer();
        if (fd < 0)
        {
            // Error polling
            _LogFile.open("RPILog.txt", std::ios::out | std::ios::app);
            _LogFile << "[" << _name << "] Error polling" << std::endl;
            _LogFile.close();
            ThreadStop();
        }
        else
        {
            // Process message
            Message * message = ReadTCP(fd);
            if (message == NULL)
            {
                // Error reading message
                continue;
            }

            int offset = 0;
            while(message->size > 0)
            {
                // Print a message
                log_message * logMessage = (log_message *)&message->buffer[offset];

                _LogFile.open("RPILog.txt", std::ios::out | std::ios::app);
                _LogFile << logMessage->entry << std::endl;
                _LogFile.close();

                // Incriment buffer pointer to read next message (if applicable)
                message->size -= sizeof(log_message);
                offset += sizeof(log_message);
            }
            free(message->buffer);
        }
    }
}

void Logger::LOG(int logFd, const char * format, ...)
{
    va_list args;
    va_start(args, format);

    log_message logData;
    vsprintf(logData.entry, format, args);
    va_end(args);

    // Use this to find out how long the log is
    //std::string str(logData.entry);
    //logData.length = str.length();

    logSend(logFd, &logData, sizeof(logData));
    
}

void Logger::logSend(int socketFd, void * data, int messageSize)
{
    // Send message (custom send function)
    int rc = send(socketFd, data, messageSize, 0); // MSG_DONTWAIT?
    if (rc <= 0)
    {
        // TODO: handle errors
        _LogFile.open("RPILog.txt", std::ios::out | std::ios::app);
        _LogFile << "[" << _name << "] An error has occured while sending data, errno: " << errno << std::endl;
        _LogFile.close();
    }
    return;
}