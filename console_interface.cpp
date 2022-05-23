//
// Created by Ja on 18.05.2022.
//

#include "console_interface.h"

/*
 * Define message queue from other thread's
 * Read data from other thread and display message in console
 * When manual mode is selected, then read data from console and send it to the trajectory generator
 */
struct console_messsage_data {
    char buffer[MAX_CONSOLE_MESSAGE_QUEUE_SZIE_BUFFOR];
    int pointer;
    pthread_cond_t empty_buffer;
    pthread_cond_t full_buffer;
    pthread_mutex_t mutex;
} console_communication_stack;

/*
 * Function to select trajectory generator mode
 * Read console input and set the global variable: robot_trajectory_mode from trajectory_generator.h
 */
void select_trajectory_mode(){
    uint8_t mode = 0;   // Robot trajectory generator mode
    bool correct_mode = false;  // Bool varaible for loop terminator
    // Wait for the correct value
    while(!correct_mode)
    {
        // Display message in console
        std::cout << "Enter number to choose robot trajectory mode:"<<std::endl;
        std::cout <<"[1] - Manual mode" << std::endl;
        std::cout <<"[2] - Automatic mode" << std::endl;
        // read single char from input and reset errors
        if( ! (std::cin >> mode) ) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cerr << "Invalid input. Try again.\n";
        }
        // display a message in case of an incorrect value
        if (std::isdigit(mode) && mode - 48 > 0 && mode - 48 < 3)
            correct_mode = true; //
        else
            std::cout<< "Invalid input. Try again.\n";
    }
    // Change char number to enum number
    robot_trajectory_mode = static_cast<Trajectory_control_type> (mode - 48);
    // Display correct value
    if (robot_trajectory_mode == Trajectory_control_type::AUTO)
        std::cout << "Selected  mode: Auto"<< std::endl;
    else
        std::cout << "Selected  mode: Manual"<< std::endl;
}

/*
 * Function below display welcome message and is waiting for the appropriate mode to be specified
 */
void console_communication_initialization(){
    // Display welcome message
    std::cout <<"Welcome to robot trajectory generator!" <<std::endl;
    // Initialization message queue stack
    pthread_cond_init(&console_communication_stack.empty_buffer, NULL);
    pthread_cond_init(&console_communication_stack.full_buffer, NULL);
    pthread_mutex_init(&console_communication_stack.mutex, NULL);
    console_communication_stack.pointer = 0;
    // Select trajectory generation mode
    select_trajectory_mode();
}

void console_interface(){
    //Initialize console
    console_communication_initialization();




}


