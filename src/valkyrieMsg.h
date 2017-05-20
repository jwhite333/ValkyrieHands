// Class to define valkyrie messages
#ifndef VALKYRIE_MSG_H
#define VALKYRIE_MSG_H

#include <map>

#define MAX_MSG_SIZE 1024
#define LOG_PORT 3737
#define CONTROL_PORT 3740

// Definitions of messages used by Valkyrie 

// Command numbers
typedef enum
{
    VMSG_NULL = 0,              // 0
    VMSG_PAUSE_MOTION,          // 1
    VMSG_ROTATE_ARM,            // 2
    VMSG_TOGGLE_IMPLIMENT,      // 3
    VMSG_TEST_SERVOS,           // 4
    VMSG_STOP,                  // 5
    VMSG_MARCO_START,           // 6
    VMSG_MACRO_END,             // 7
    VMSG_COUNT                  // 8
} VMSG_TYPES;

// Command abbreviations
extern std::map<std::string, int> commandMap;

// Command descriptions
extern std::string VMSG_NAMES[VMSG_COUNT];

// Message sizes
extern std::map<VMSG_TYPES, int> messageSizes;

/**
* Header
*
**/
typedef struct
{
    uint16_t type;
    uint16_t length;
}
VMSG_HEADER_T;

/**
* Pause Motion
*
**/
typedef struct
{
    VMSG_HEADER_T hdr;
    int duration;
} VMSG_PAUSE_MOTION_T;

/**
* Rotate Arm
*
**/
typedef struct
{
    VMSG_HEADER_T hdr;
    int direction;
    int speed;
    int degrees;
} VMSG_ROTATE_ARM_T;

/**
* Toggle Impliment
*
**/
typedef struct
{
    VMSG_HEADER_T hdr;
    bool power;
} VMSG_TOGGLE_IMPLIMENT_T;

/**
* Test Servos
*
**/
typedef struct
{
    VMSG_HEADER_T hdr;
    int duration;
} VMSG_TEST_SERVOS_T;

/**
* Stop
*
**/
typedef struct
{
    VMSG_HEADER_T hdr;
    void * shutdown_info; // Null unless needed in the future
} VMSG_STOP_T;

/**
* Macro Start
* 
**/
typedef struct
{
    VMSG_HEADER_T hdr;
    void * info;
} VMSG_MACRO_START_T;

/**
* Macro End
* 
**/
typedef struct
{
    VMSG_HEADER_T hdr;
    void * info;
} VMSG_MACRO_END_T;

#endif