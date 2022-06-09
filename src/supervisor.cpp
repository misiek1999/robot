//
// Created by Ja on 17.05.2022.
//

#include "supervisor.h"

// Program state variable
std::atomic <ProgramState> program_state;

// Function to close program
static void exit_handler(int){
    set_program_state(ProgramState::CLOSE_PROGRAM); // set close state in program
    // Send message to log and console
    write_to_console("CRTL+C detected, stop program!");
    write_to_log("CRTL+C detected, stop program!");
}
// Get program state
const ProgramState get_program_state(){
    return program_state;
}

// Set program state
void set_program_state(const ProgramState _state_to_set){
    program_state = _state_to_set;
}

// Supervisor thread function
void * program_supervisor(void *pVoid){
    // Init thread priority
    int policy;     //Scheduling policy: FIFO or RR
    struct sched_param param;   //Structure of other thread parameters

    /* Read modify and set new thread priority */
    pthread_getschedparam( pthread_self(), &policy, &param);
    param.sched_priority = sched_get_priority_max(policy);  // Read minimum value for thread priority
    pthread_setschedparam( pthread_self(), policy, &param);   //set maximum thread priority for this thread

    // setup signal SIGINT handler and quit function
    struct sigaction close_act;
    close_act.sa_handler = exit_handler;    // set close function handle
    sigemptyset(&close_act.sa_mask);        // init signal struct with zeros
    close_act.sa_flags = 0;

    // Register signal handler for SIGINT
    if (sigaction(SIGINT, &close_act, NULL) < 0) {
        fprintf(stderr, "Cannot register SIGINT handler.\n");
        return 0;
    }

//    // Create empty signal set to run sigsuspend
//    sigset_t mask;
//    sigemptyset(&mask);
//    sigprocmask(0, NULL, &mask);
//    //TODO: finish this function
//    for (int i = 2; i < SIGRTMAX; i++)
//        sigaddset(&mask, SIGRTMIN+i);
//    sigprocmask(SIG_SETMASK, &mask, NULL);
//
//    /* Prepare sigaction struct for handler */
//    struct sigaction task_action;
//    task_action.sa_handler = task_handler;
//    sigemptyset(&task_action.sa_mask);
//    task_action.sa_flags = 0;




    return 0;
}