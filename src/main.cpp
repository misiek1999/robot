/*
 * This is the main function of robot trajectory generator
 * Include all necesery liblaries, variables and function
 * Launch the following threads:
 * 1 - Supervisor
 * 2 - Robot internet interface
 * 3 - Trajecotry generator
 * 4 - Console interface
 * 5 - File log 
 */
// Include libraries
#include <iostream>
#include <semaphore.h>
#include "mutex"
#include "pthread.h"
#include "signal.h"
#include "stdlib.h"

#include "supervisor.h"
#include "controll_interface.h"
#include "trajectory_generator.h"
#include "console_interface.h"
#include "logger.h"
#include "mqueue.h"

// Function to setup all message queue's in program
// Return true is setup is completed or false when setup is failed
bool setup_all_mes_queues();

// Setup and run threads
bool launch_threads();

// Thread variables
pthread_t supervisor_thread;
pthread_t control_thread;
pthread_t trajectory_thread;
pthread_t console_thread;
pthread_t log_thread;
// Thread attributes variable
pthread_attr_t supervisor_thread_attr;
pthread_attr_t control_thread_attr;
pthread_attr_t trajectory_thread_attr;
pthread_attr_t console_thread_attr;
pthread_attr_t log_thread_attr;

// Main function
int main() {
    // Clear console buffer
    std::cin.clear();
    fflush(stdin);
    // Display welcome message
   std::cout << "Initialization bitcoin miner..." << std::endl;
    // Initialize all message queue in main thread before launching other thread to avoid access error
    if (!setup_all_mes_queues())// If initialization failed then stop application
        exit(EXIT_FAILURE);

    // Init and launch threads
    if (!launch_threads())// If initialization failed then stop application
        exit(EXIT_FAILURE);

    

    log_string("Witam");


    /* Wait for killing signal */
    sigset_t empty;
    sigemptyset(&empty);

    sigsuspend(&empty);


    set_program_state(ControllerState::CLOSE_PROGRAM);
    sleep(1);
    // Close all message queue's
    mq_close(mes_to_logger_queue);
    mq_close(mes_to_console_queue);
    mq_close(mes_to_trajectory_queue);
    return 0;
}

bool launch_threads(){
    /* Scheduling policy: FIFO or RR */
    int policy;
    /* Structure of other thread parameters */
    struct sched_param param;
    /* Initialize thread attributes structure for FIFO scheduling */
    pthread_attr_init(&supervisor_thread_attr);
    pthread_attr_init(&control_thread_attr);
    pthread_attr_init(&trajectory_thread_attr);
    pthread_attr_init(&console_thread_attr);
    pthread_attr_init(&log_thread_attr);
    pthread_attr_setschedpolicy(&supervisor_thread_attr, SCHED_FIFO);
    pthread_attr_setschedpolicy(&control_thread_attr, SCHED_FIFO);
    pthread_attr_setschedpolicy(&trajectory_thread_attr, SCHED_FIFO);
    pthread_attr_setschedpolicy(&console_thread_attr, SCHED_FIFO);
    pthread_attr_setschedpolicy(&log_thread_attr, SCHED_FIFO);

    int thread_create_status;// Status of creating threads
    // Create threads
    // If creating fail, then return false
    if ((thread_create_status = pthread_create( &supervisor_thread, &supervisor_thread_attr, program_supervisor, nullptr))) {
        fprintf(stderr, "Cannot create thread.\n");
        return false;
    }
//    if ((thread_create_status = pthread_create( &control_thread, &control_thread_attr, , nullptr))) {
//        fprintf(stderr, "Cannot create thread.\n");
//        return false;
//    }
//    if ((thread_create_status = pthread_create( &t, &supervisor_thread_attr, program_supervisor, nullptr))) {
//        fprintf(stderr, "Cannot create thread.\n");
//        return false;
//    }
    if ((thread_create_status = pthread_create( &console_thread, &console_thread_attr, console_interface, nullptr))) {
        fprintf(stderr, "Cannot create thread.\n");
        return false;
    }
    if ((thread_create_status = pthread_create( &log_thread, &log_thread_attr, log_data_to_file, nullptr))) {
        fprintf(stderr, "Cannot create thread.\n");
        return false;
    }


    return true;
}

bool setup_all_mes_queues(){
    // Setup message queue from trajectory to console
    mes_to_console_queue_attr.mq_maxmsg = MAX_MESSAGES_IN_QUEUE;    //Max 32 messages in queue
    mes_to_console_queue_attr.mq_msgsize = sizeof(meq_que_data_t);  //Char buffer for 32 characters
    mes_to_console_queue_attr.mq_flags = O_NONBLOCK;
    // Create message queue
    if ((mes_to_console_queue = mq_open("/mesQueCons", O_CREAT | O_RDWR, 0644, &mes_to_console_queue_attr)) == -1) {
        fprintf(stderr, "Creation of the mes queues failed 1\n");
        return false;
    }
    // Setup message queue from console to logger
    mes_to_logger_queue_attr.mq_maxmsg = MAX_MESSAGES_IN_QUEUE;    //Max 32 messages in queue
    mes_to_logger_queue_attr.mq_msgsize = sizeof(meq_que_data_t);  //Char buffer for 32 characters
    mes_to_logger_queue_attr.mq_flags = O_NONBLOCK;
    // Create message queue
    if ((mes_to_logger_queue = mq_open("/meqQueLog", O_CREAT | O_RDWR, 0644, &mes_to_logger_queue_attr)) == -1) {
        fprintf(stderr, "Creation of the mes queues failed 2\n");
        return false;
    }
    // Setup message queue from console to trajectory
    mes_to_trajectory_queue_attr.mq_maxmsg = MAX_MESSAGES_IN_QUEUE;    //Max 32 messages in queue
    mes_to_trajectory_queue_attr.mq_msgsize = sizeof(meq_que_data_t);  //Char buffer for 32 characters
    mes_to_trajectory_queue_attr.mq_flags = O_NONBLOCK;
    // Create message queue
    if ((mes_to_trajectory_queue = mq_open("/mes_que_traj", O_CREAT | O_RDWR, 0644, &mes_to_trajectory_queue_attr)) == -1) {
        fprintf(stderr, "Creation of the mes queues failed 3\n");
        return false;
    }
    // If setup is successful completed then return true
    return true;
}