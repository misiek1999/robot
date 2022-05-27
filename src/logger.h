//
// Created by Ja on 16.05.2022.
//
// This file include all functions which save data to file
#ifndef ROBOT_LOGGER_H
#define ROBOT_LOGGER_H
#include "iostream"
#include "fstream"
#include "time.h"
#include "chrono"
#include "stdlib.h"
#include "string.h"
#include "mqueue.h"
#include "supervisor.h"

// Message queue with data to save into file
extern mqd_t mes_to_logger_queue;   // Message queue
extern struct	mq_attr mes_to_logger_queue_attr;
typedef char log_mes_que_data_t[20];    // received data type

// Function to send string to log
void log_string(const  char* str_to_log);
// Logging data to file from message queue
void * log_data_to_file(void *pVoid);



#endif //ROBOT_LOGGER_H

