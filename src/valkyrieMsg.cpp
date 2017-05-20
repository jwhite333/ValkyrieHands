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