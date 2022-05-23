//
// Created by Ja on 18.05.2022.
//

#ifndef ROBOT_CONSOLE_INTERFACE_H
#define ROBOT_CONSOLE_INTERFACE_H
// Include libraries
#include "iostream"
#include "fstream"
#include "ctype.h"
#include "stdio.h"
#include "pthread.h"
#include "string"
#include "unistd.h"
#include "controll_interface.h"
#include "trajectory_generator.h"
// Global constance variable
 #define MAX_CONSOLE_MESSAGE_QUEUE_SIZE_BUFFER 128

// Global message queue to console
 extern struct console_message_data console_communication_stack;

// Define function
void console_interface();


#endif //ROBOT_CONSOLE_INTERFACE_H
