// Class for message logic handling
#ifndef LOGIC_HANDLER_H
#define LOGIC_HANDLER_H

#include "thread.h"
#include "messageHandler.h"
#include "valkyrieMsg.h"
#include "servoController.h"
#include "logger.h"

#define SERVO_COUNT 2

class LogicHandler : public Thread, public messageHandler
{
    public:
        // Initializers
        void ThreadEntry(void); // Overwrites Thread.h
        void pollCommands();

    private:
        typedef void (LogicHandler::*CallbackFunction)(void *);
        std::map<VMSG_TYPES, CallbackFunction> _callbackMap;
        void handlePauseMotion(void * buffer);
        void handleStop(void * buffer);
        std::shared_ptr<ServoController> _servoControllers[SERVO_COUNT];
        int _servoFd[SERVO_COUNT];
        Logger logger;
        int _logFd;
};

#endif