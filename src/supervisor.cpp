//
// Created by Ja on 17.05.2022.
//

#include "supervisor.h"

// Program state variable
std::atomic <ProgramState> program_state;

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


    return 0;
}