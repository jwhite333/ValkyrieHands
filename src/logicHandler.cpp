// Class Definitions for message logic handling
#include "logicHandler.h"

void LogicHandler::ThreadEntry(void)
{
    // Connect to logging thread
    _logFd = ConnectToTCPServer("127.0.0.1", LOG_PORT);
    if (_logFd < 0)
    {
        // Error connecting to logger thread
        std::cout << "Error connecting to logger thread" << std::endl;
        ThreadStop();
    }

    VMSG_LOG_T log("Thread: " + _name + " started");
    SendTCP(_logFd, &log, 0, sizeof(log));
    //std::cout << "Thread: " << _name << " started" << std::endl;

    // Initialize callback map
    _callbackMap[VMSG_PAUSE_MOTION] = &LogicHandler::handlePauseMotion;
    _callbackMap[VMSG_STOP] = &LogicHandler::handleStop;

    // Start TCP server
    int rc = StartTCPServer(_port);
    if (rc < 0)
    {
        // Starting server failed
        std::cout << "Starting server failed" << std::endl;
        ThreadStop();
    }

    // Start servo threads
    std::cout << "Starting servo threads" << std::endl;
    for (int servo_num = 0; servo_num < SERVO_COUNT; servo_num++)
    {
        std::stringstream ss;
        ss << servo_num;
        std::string str = ss.str();
        std::string threadName = "Servo" + str;
        _servoControllers[servo_num] = std::make_shared<ServoController>();
        _servoControllers[servo_num]->ThreadStart(threadName, (CONTROL_PORT + 1) + servo_num); // 3741 - 3750
    }
    sleep(1);

    // Connect to servo threads
    for (int servo_num = 0; servo_num < SERVO_COUNT; servo_num++)
    {
        _servoFd[servo_num] = ConnectToTCPServer("127.0.0.1", (CONTROL_PORT + 1) + servo_num);
        if (_servoFd[servo_num] < 0)
        {
            // Error connecting to socket
            std::cout << "Error connecting to Servo thread " << servo_num << std::endl;
            ThreadStop();
        }
    }
    

    pollCommands();
}

void LogicHandler::pollCommands()
{
    int fd;
    while (_running)
    {
        fd = PollTCPServer();
        if (fd < 0)
        {
            // Error polling
            std::cout << "Error polling" << std::endl;
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

            // Determine data type
            VMSG_HEADER_T * header = (VMSG_HEADER_T *)buffer;
            VMSG_TYPES messageType = (VMSG_TYPES)header->type;

            // Process data
            (this->*_callbackMap[messageType])(buffer);            
            free(buffer);
        }
    }
}

void LogicHandler::handlePauseMotion(void * buffer)
{
    // Handle message
    VMSG_PAUSE_MOTION_T * pause = (VMSG_PAUSE_MOTION_T *)buffer;
    std::cout << "Received Pause Motion, duration: " << pause->duration << std::endl;

    // Forward to servo thread
    for (int servo_num = 0; servo_num < SERVO_COUNT; servo_num++)
    {
        SendTCP(_servoFd[servo_num], buffer, VMSG_PAUSE_MOTION, sizeof(VMSG_PAUSE_MOTION_T));
    }
    return;
}

void LogicHandler::handleStop(void * buffer)
{
    // Handle message
    VMSG_STOP_T * stop = (VMSG_STOP_T *)buffer;

    // Forward to servo thread
    for (int servo_num = 0; servo_num < SERVO_COUNT; servo_num++)
    {
        SendTCP(_servoFd[servo_num], buffer, VMSG_STOP, sizeof(VMSG_STOP_T));
    }
    return;
    ThreadStop();
}