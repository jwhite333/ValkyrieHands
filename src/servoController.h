// Class for sending ROS messages to servo motors
#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include "thread.h"
#include "messageHandler.h"
#include "valkyrieMsg.h"
#include "logger.h"

class ServoController : public Thread, public messageHandler
{
    public:
        // Initializers
        void ThreadEntry(void); // Overwrites Thread.h
        void pollCommands();

    private:
        typedef void (ServoController::*CallbackFunction)(void *);
        std::map<VMSG_TYPES, CallbackFunction> _callbackMap;
        void handlePauseMotion(void * buffer);
        void handleStop(void * buffer);
        Logger logger;
        int _logFd;
};

#endif