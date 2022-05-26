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
#include "supervisor.h"
#include "termios.h"
#include "mqueue.h"
#include "sys/fcntl.h"
#include "stdlib.h"

// Global constance variable
 #define MAX_CONSOLE_MESSAGE_QUEUE_SIZE_BUFFER 128

// Global message queue to console
 extern mqd_t	mes_to_console_queue;
extern struct	mq_attr mes_to_console_queue_attr;

// Message queue buffer type and queue size
#define MAX_MESSAGES_IN_QUEUE 32
typedef char meq_que_data_t[32];

// Define function which interface with console
// Run this task on low priority thread
void * console_interface(void *pVoid);


#endif //ROBOT_CONSOLE_INTERFACE_H
