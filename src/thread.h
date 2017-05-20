// Class for handling threads
#ifndef THREAD_H
#define THREAD_H

#include <string>
#include <iostream>
#include <sstream>
#include <syslog.h>
#include <pthread.h>
#include <memory>

/*
Thread sockets

Logic Thread: 3740
Servo Controllers: 3741 - 3750

*/

/**
* Thread
*
**/
class Thread
{
    public:
        virtual int ThreadStart(std::string threadName, int bindOnPort)
        {
            _running  = true;
            _name = threadName;
            _port = bindOnPort;

            int rc = pthread_create(&_thread, NULL, ThreadMain, (void *)this);
            if (rc)
                std::cout << "Error starting thread" << std::endl;

            return rc;
        }
        virtual void ThreadStop(void)
        {
            std::cout << "Stopping Thread: " << _name << std::endl;
            pthread_exit(NULL);
            return;
        }

    protected:
        bool _running;
        std::string _name;
        int _port;

    private:
        static  void* ThreadMain(void* thread)
        {
            Thread* newThread = (Thread*) thread;

            newThread->ThreadEntry();

            pthread_exit(NULL);
            return 0;
        }

        virtual void ThreadEntry(void)= 0;

        pthread_t _thread;
};

#endif
