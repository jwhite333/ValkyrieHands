// Class Definitions for message logic handling (open and close log file each time to prevent missed data on ctrl+c exit)

#include "logger.h"

void Logger::ThreadEntry(void)
{
    _LogFile.open("RPILog.txt", std::ios::out | std::ios::trunc); // Create a new file each time

    _LogFile << "Thread: " << _name << " started" << std::endl;
    std::cout << "Thread: " << _name << " started" << std::endl;

    // Start TCP server
    int rc = StartTCPServer(_port);
    if (rc < 0)
    {
        // Starting server failed
        _LogFile << "Starting server failed" << std::endl;
        std::cout << "Starting server failed" << std::endl;
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
            _LogFile << "Error polling" << std::endl;
            std::cout << "Error polling" << std::endl;
            _LogFile.close();
            ThreadStop();
        }
        else
        {
            // Process message
            char * buffer = (char *)ReadTCP(fd);
            if (buffer == NULL)
            {
                // Error reading message
                continue;
            }

            // Write log TODO: check length
            VMSG_LOG_T * logMessage = (VMSG_LOG_T *)buffer;
            _LogFile.open("RPILog.txt", std::ios::out | std::ios::app);
            _LogFile << logMessage->log_entry << std::endl;
            std::cout << logMessage->log_entry << std::endl;
            _LogFile.close();           
            free(buffer);
        }
    }
}