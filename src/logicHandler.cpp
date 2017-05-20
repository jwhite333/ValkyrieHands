// Class Definitions for message logic handling
#include "logicHandler.h"

void LogicHandler::ThreadEntry(void)
{
    // Connect to logging thread
    _logFd = ConnectToTCPServer("127.0.0.1", LOG_PORT);
    if (_logFd < 0)
    {
        // Error connecting to logger thread
        std::cout << "[" << _name << "] Error connecting to logger thread" << std::endl;
        ThreadStop();
    }
    logger.LOG(_logFd, "[%s] Thread started", _name.c_str());
    logger.LOG(_logFd, "[%s] Connected to logger at: %s:%d", _name.c_str(), "127.0.0.1", LOG_PORT);

    // Initialize callback map
    _callbackMap[VMSG_PAUSE_MOTION] = &LogicHandler::handlePauseMotion;
    _callbackMap[VMSG_STOP] = &LogicHandler::handleStop;

    // Start TCP server
    int rc = StartTCPServer(_serverPort);
    if (rc < 0)
    {
        // Starting server failed
        logger.LOG(_logFd, "[%s] Starting server failed", _name.c_str());
        ThreadStop();
    }
    logger.LOG(_logFd, "[%s] Started server on port %d", _name.c_str(), _serverPort);

    // Start servo threads
    logger.LOG(_logFd, "[%s] Starting servo threads", _name.c_str());
    for (int servo_num = 0; servo_num < SERVO_COUNT; servo_num++)
    {
        std::stringstream ss;
        ss << servo_num;
        std::string str = ss.str();
        std::string threadName = "Servo" + str;
        _servoControllers[servo_num] = std::make_shared<ServoController>();
        _servoControllers[servo_num]->ThreadStart(threadName, (CONTROL_PORT + 1) + servo_num); // 3741 - 3750
    }
    //sleep(2);

    // Connect to servo threads
    for (int servo_num = 0; servo_num < SERVO_COUNT; servo_num++)
    {
        _servoFd[servo_num] = ConnectToTCPServer("127.0.0.1", (CONTROL_PORT + 1) + servo_num);
        if (_servoFd[servo_num] < 0)
        {
            // Error connecting to socket
            logger.LOG(_logFd, "[%s] Error connecting to ServoController %d", _name.c_str(), (CONTROL_PORT + 1) + servo_num);
            ThreadStop();
        }
        logger.LOG(_logFd, "[%s] Connected to ServoController on port %d", _name.c_str(), (CONTROL_PORT + 1) + servo_num);
    }
    

    pollCommands();
}

void LogicHandler::pollCommands()
{
    int fd;
    while (_running)
    {
        fd = PollTCPServer();
        if (fd == -1)
        {
            // Error polling
            logger.LOG(_logFd, "[%s] Error polling", _name.c_str());
            ThreadStop();
        }
        else if (fd == -2)
        {
            // Timeout
            continue;
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
                // Determine data type
                VMSG_HEADER_T * header = (VMSG_HEADER_T *)message->buffer;
                VMSG_TYPES messageType = (VMSG_TYPES)header->type;

                // Process data
                (this->*_callbackMap[messageType])(message->buffer);

                // Incriment buffer pointer to read next message (if applicable)
                message->size -= messageSizes[messageType];
                offset += messageSizes[messageType];
            }
            FreeMessage(message);
        }
    }
}

void LogicHandler::handlePauseMotion(void * buffer)
{
    // Handle message
    VMSG_PAUSE_MOTION_T * pause = (VMSG_PAUSE_MOTION_T *)buffer;
    logger.LOG(_logFd, "[%s] Received pause motion command, duration: %d", _name.c_str(), pause->duration);

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
    logger.LOG(_logFd, "[%s] Stopping thread", _name.c_str());
    ThreadStop();
    return;
}