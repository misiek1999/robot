//
// Created by Ja on 17.05.2022.
//

#include "supervisor.h"

// Program state variable
std::atomic <ControllerState> program_state;

// Get program state
const ControllerState get_program_state(){
    return program_state;
}

// Set program state
void set_program_state(const ControllerState _state_to_set){
    program_state = _state_to_set;
}

// Supervisor thread function
void * program_supervisor(void *pVoid){
    std::cout << "Test";
    return 0;
}