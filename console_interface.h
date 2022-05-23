//
// Created by Ja on 18.05.2022.
//

#ifndef ROBOT_CONSOLE_INTERFACE_H
#define ROBOT_CONSOLE_INTERFACE_H
    // Include liblaries
    #include "iostream"
    #include "ctype.h"
    #include "stdio.h"
    #include "pthread.h"
    #include "controll_interface.h"
    #include "trajectory_generator.h"
    // Global constance variable
     #define MAX_CONSOLE_MESSAGE_QUEUE_SZIE_BUFFOR 128

    // Global message queue to console
     extern struct console_messsage_data console_communication_stack;

    // Define function
    void console_interface();


#endif //ROBOT_CONSOLE_INTERFACE_H
