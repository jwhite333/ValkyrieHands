// Class for message logic handling
#ifndef LOGGER_H
#define LOGGER_H

#include <fstream>

#include "thread.h"
#include "messageHandler.h"
#include "valkyrieMsg.h"

class Logger : public Thread, public messageHandler
{
    public:
        // Initializers
        void ThreadEntry(void); // Overwrites Thread.h
        void pollCommands();
    private:
        std::ofstream _LogFile;
};

#endif