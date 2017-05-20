// Class Definitions for sending ROS messages to servo motors
#include "servoController.h"

void ServoController::ThreadEntry(void)
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

    // Initialize ROS data
    /*int argc = 0;
    ros::init(argc, NULL, "ROS_RPI");
    ros::NodeHandle rosNode;
    ros::Publisher rpi_pub = rosNode.advertise<std_msgs::String>(_name, 1000);
    ros::Rate loop_rate(10);*/

    // Initialize callback map
    _callbackMap[VMSG_PAUSE_MOTION] = &ServoController::handlePauseMotion;
    _callbackMap[VMSG_STOP] = &ServoController::handleStop;

    // Start TCP server
    int rc = StartTCPServer(_serverPort);
    if (rc < 0)
    {
        // Starting server failed
        logger.LOG(_logFd, "[%s] Starting server failed", _name.c_str());
        ThreadStop();
    }
    logger.LOG(_logFd, "[%s] Started server on port %d", _name.c_str(), _serverPort);

    pollCommands();
}

void ServoController::pollCommands()
{
    //logger.LOG(_logFd, "[%s] Thread started", threadName.c_str());
    int fd;
    while (_running)
    {
        fd = PollTCPServer(1); // 1/1000 second timeout
        if (fd == -1)
        {
            // Error polling
            logger.LOG(_logFd, "[%s] Error polling", _name.c_str());
            ThreadStop();
        }
        else if (fd == -2)
        {
            // Timeout, perform ROS sends here
            logger.LOG(_logFd, "[%s] Sample servo value packet", _name.c_str());
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

void ServoController::handlePauseMotion(void * buffer)
{
    // Handle message
    VMSG_PAUSE_MOTION_T * pause = (VMSG_PAUSE_MOTION_T *)buffer;
    logger.LOG(_logFd, "[%s] Received pause motion command, duration: %d", _name.c_str(), pause->duration);
    return;
}

void ServoController::handleStop(void * buffer)
{
    // Stop ROS
    ros::shutdown();
    
    // Handle message
    VMSG_STOP_T * stop = (VMSG_STOP_T *)buffer;
    logger.LOG(_logFd, "[%s] Stopping thread", _name.c_str());
    ThreadStop();
}