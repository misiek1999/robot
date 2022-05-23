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
// Include liblaries
#include <iostream>
#include <semaphore>
#include "mutex"
#include "thread"
#include "pthread.h"
#include "signal.h"
#include "console_interface.h"



int main() {
    std::cout << "Initialization bitcoin miner..." << std::endl;



    console_interface();
    return 0;
}
