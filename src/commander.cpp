// Main controller for rpi
#include "valkyrieMsg.h"
#include "messageHandler.h"
#include "logicHandler.h"
#include "thread.h"
#include "logger.h"

// Initialize global message handler
messageHandler msgHandler;

/**
* Validate Command
* 
**/
int validateCommand(std::string command)
{
    if (commandMap.find(command) != commandMap.end())
    {
        return commandMap[command];
    }
    else
    {
        std::cout << "Invalid Command\n";
        return -1;
    }
}

/**
* Main
*
**/
int main(int argc, char **argv)
{
    // Initialize logic thread and logger thread
    std::shared_ptr<Logger> logger = std::make_shared<Logger>();
    logger->ThreadStart("Logger", LOG_PORT);
    sleep(1);
    std::shared_ptr<LogicHandler> logic = std::make_shared<LogicHandler>();
    logic->ThreadStart("LogicHandler", CONTROL_PORT);
    //sleep(1);

    // Display info message
    int msgCount = 0;
    std::cout << "Enter Command ( \"Help\" - For list of commands ) \n";

    // Connect to Logic Thread
    int socketFd = msgHandler.ConnectToTCPServer("127.0.0.1", CONTROL_PORT);
    if (socketFd < 0)
    {
        // Error connecting to socket
        std::cout << "Error connecting to logic handler server" << std::endl;
    }

    while (true) // ros::ok()
    {
        // Get command
        std::string commandString = "";
        int command = -1;
        while (command < 0)
        {
            std::cout << "Enter command: ";
            std::cin >> commandString;
            command = validateCommand(commandString);
        }

        // Create stringstream
        //std::stringstream ss;

        // Execute command
        switch (command)
        {
        case VMSG_PAUSE_MOTION:
        {
            VMSG_PAUSE_MOTION_T pause;
            pause.duration = -1;
            msgHandler.SendTCP(socketFd, &pause, VMSG_PAUSE_MOTION, sizeof(VMSG_PAUSE_MOTION_T));
        }
        break;

        case VMSG_STOP:
        {
            VMSG_STOP_T stop;
            stop.shutdown_info = NULL;
            msgHandler.SendTCP(socketFd, &stop, VMSG_STOP, sizeof(VMSG_STOP_T));
            break;
        }
        break;

        default:
        {
            // Default
        }
        break;
        }

        //std_msgs::String msg;
        //ss << commandString << msgCount;
        //msg.data = ss.str();

        //ROS_INFO("%s", msg.data.c_str());
        //rpi_pub.publish(msg);

        //ros::spinOnce();
        //loop_rate.sleep();
        ++msgCount;
    }

    // Wait for logic thread to stop
    //pthread_join(logicThread, NULL);
    logic->ThreadStop();

    return 0;
}
