//
// Created by Ja on 17.05.2022.
//

#include "supervisor.h"

// Program state variable
std::atomic <ProgramState> program_state;

// Function to close program
static void exit_handler(int input_signal){
    set_program_state(ProgramState::CLOSE_PROGRAM); // set close state in program
    // Send message to log and console
    write_to_console("Stop signal handle!");
    write_to_log("Stop signal handle!");
}

// Function for signal handler
static void emergency_stop_handler(int input_signal){
    set_program_state(ProgramState::EMERGENCY_STOP);
}

// Get program state
const ProgramState get_program_state(){
    return program_state;
}

// Set program state
void set_program_state(const ProgramState _state_to_set){
    program_state = _state_to_set;
    // Check if selected state is CLOSE_PROGRAM
    if (program_state == ProgramState::CLOSE_PROGRAM)
        raise(CLOSE_PROGRAM_SIGNAL);    // raise close program signal to supervisor thread
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

    // Create empty signal set to run sigsuspend
    sigset_t supervisor_mask;
    sigemptyset(&supervisor_mask);
    sigprocmask(0, NULL, &supervisor_mask);

    // Add signals to supervisor_mask
    sigaddset(&supervisor_mask, CLOSE_PROGRAM_SIGNAL);     // Signal used to close program
    sigaddset(&supervisor_mask, EMERGENCY_STOP_SIGNAL);    // Signal used to emergency stop program
    sigprocmask(SIG_SETMASK, &supervisor_mask, NULL);

    // Prepare signal action struct for function handler
    struct sigaction emergency_stop_action;
    emergency_stop_action.sa_handler = emergency_stop_handler;
    sigemptyset(&emergency_stop_action.sa_mask);
    emergency_stop_action.sa_flags = 0;

    // Register signal handler for EMERGENCY STOP SIGNAL
    if (sigaction(EMERGENCY_STOP_SIGNAL, &emergency_stop_action, NULL) < 0) {
        fprintf(stderr, "Cannot register EMERGENCY STOP handler.\n");
        return 0;
    }

    // Enter to loop and wake up only receive any signal
    while(get_program_state() != ProgramState::CLOSE_PROGRAM){
        sigsuspend(&supervisor_mask);
    }

    return 0;
}