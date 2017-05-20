// Class Definitions for sending ROS messages to servo motors
#include "servoController.h"

void ServoController::ThreadEntry(void)
{
    std::cout << "Thread: " << _name << " started" << std::endl;

    // Initialize ROS data
    int argc = 0;
    //ros::init(argc, NULL, "ROS_RPI");
    //ros::NodeHandle rosNode;
    //ros::Publisher rpi_pub = rosNode.advertise<std_msgs::String>(_name, 1000);
    //ros::Rate loop_rate(10);

    // Initialize callback map
    _callbackMap[VMSG_PAUSE_MOTION] = &ServoController::handlePauseMotion;
    _callbackMap[VMSG_STOP] = &ServoController::handleStop;

    // Start TCP server
    int rc = StartTCPServer(_port);
    if (rc < 0)
    {
        // Starting server failed
        std::cout << "Starting server failed" << std::endl;
        ThreadStop();
    }

    pollCommands();
}

void ServoController::pollCommands()
{
    int fd;
    while (_running)
    {
        fd = PollTCPServer(1); // 1/1000 second timeout
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

void ServoController::handlePauseMotion(void * buffer)
{
    // Handle message
    VMSG_PAUSE_MOTION_T * pause = (VMSG_PAUSE_MOTION_T *)buffer;
    std::cout << "Received Pause Motion, duration: " << pause->duration << std::endl;
    return;
}

void ServoController::handleStop(void * buffer)
{
    // Handle message
    VMSG_STOP_T * stop = (VMSG_STOP_T *)buffer;
    ThreadStop();
}