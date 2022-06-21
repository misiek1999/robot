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
#include "instruction_processor.h"
#include "read_instruction_file.h"
#include "controll_interface.h"
#include "trajectory_generator.h"
#include "supervisor.h"
#include "logger.h"
#include "termios.h"
#include "mqueue.h"
#include "sys/fcntl.h"
#include "stdlib.h"
#include "math.h"


// Signal used to communicate with console
#define SIGNAL_STOP_CONSOLE SIGRTMIN+1
#define SIGNAL_EMERGENCY_STOP_CONSOLE SIGRTMIN+2

// Global message queue to console
 extern mqd_t	mes_to_console_queue;
extern struct	mq_attr mes_to_console_queue_attr;

// Message queue buffer type and queue size
#define MAX_MESSAGES_IN_QUEUE 10
typedef char mq_consol_data_t[100];

// Define function which interface with console
// Run this task on low priority thread
void * console_interface(void *pVoid);

/*
 * Interface with console
 * Send cstring to message queue, which is later displayed in console
*/
void write_to_console(const  char* str_to_console);

#endif //ROBOT_CONSOLE_INTERFACE_H
