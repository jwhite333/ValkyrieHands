// Class for message logic handling (in ROS this file is in ~ directory)
#ifndef LOGGER_H
#define LOGGER_H

#define MAX_LOG_SIZE 256

#include <fstream>
#include <cstdarg>
#include <stdarg.h>
#include <stdio.h>

#include "thread.h"
#include "messageHandler.h"
#include "valkyrieMsg.h"

class Logger : public Thread, public messageHandler
{
    public:
        // Initializers
        void ThreadEntry(void); // Overwrites Thread.h
        void pollCommands();
        void LOG(int logFd, const char * format, ...);
    private:
        void logSend(int socketFd, void * data, int messageSize);
        std::ofstream _LogFile;
        struct log_message
        {
            int length;
            char entry[MAX_LOG_SIZE];
        };
};

#endif