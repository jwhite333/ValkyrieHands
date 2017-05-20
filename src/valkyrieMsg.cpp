// Definitions for objects in alkyrieMsg.h
#include "valkyrieMsg.h"

// Command map
std::map<std::string, int> commandMap =
    {
        {"PSE", 1},
        {"RTA", 2},
        {"TGI", 3},
        {"TST", 4},
        {"STP", 5},
        {"MCS", 6},
        {"MCE", 7}};

// Message names
std::string VMSG_NAMES[VMSG_COUNT] =
    {
        "NULL",
        "Pause Motion",
        "Rotate Arm",
        "Toggle Impliment On/Off",
        "Test Servos",
        "Stop",
        "Macro Start",
        "Macro End"};

std::map<VMSG_TYPES, int> messageSizes = 
    {
        { VMSG_NULL, 0},
        { VMSG_PAUSE_MOTION, sizeof(VMSG_PAUSE_MOTION_T) },
        { VMSG_ROTATE_ARM, sizeof(VMSG_ROTATE_ARM_T) },
        { VMSG_TOGGLE_IMPLIMENT, sizeof(VMSG_TOGGLE_IMPLIMENT_T) },
        { VMSG_TEST_SERVOS, sizeof(VMSG_TEST_SERVOS_T) },
        { VMSG_STOP, sizeof(VMSG_STOP_T) },
        { VMSG_MARCO_START, sizeof(VMSG_MACRO_START_T) },
        { VMSG_MACRO_END, sizeof(VMSG_MACRO_END_T) }};