//
// Created by Ja on 18.05.2022.
//

#include "console_interface.h"

/*
 * Define message queue from other thread's
 * Read data from other thread and display message in console
 * When manual mode is selected, then read data from console and send it to the trajectory generator
 * This structure is default in posix message queue
 */
struct console_message_data {
    char buffer[MAX_CONSOLE_MESSAGE_QUEUE_SIZE_BUFFER];
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
    bool correct_mode = false;  // Bool variable for loop terminator
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
 * Function read data from queue and display it on console
 */
void display_queue_messages(){



}

/*
 * Function to read correct file path from console
 */
std::string read_file_path_from_console(){
    std::string path_to_file;
    std::cout << "Enter the full path of the file" << std::endl;    // Display path request message
    bool path_is_correct = false;   // bool variable to stop reading data from console
    int res;    // variable to check file is exist
    while(!path_is_correct){
        std::cin >> path_to_file; // read file path from console
        // check file exist
        res = access(path_to_file.c_str(), R_OK);   //check file is exist
        if (res == 0)   // If res value is non-negative then path is correct
            path_is_correct = true;
        else
            std::cout<<"Invalid path! Please specified valid path." <<std::endl;
    }
    std::cout<<"Specified path is correct. Path: " <<path_to_file<<std::endl;
    return path_to_file;
}

/*
 * File to read predefined trajectory from file
 * also makes the read data available to the trajectory generator
 */
void read_trajectory_from_file(std::string _path_to_file){
    // Open file
    std::ifstream file;
    file.open(_path_to_file);
    if (file.is_open()) // Check is file opened
    {
        // temp iterator
        size_t instr_itr = 0;
        // temp instruction
        TrajectoryInstruction temp_inst;
        // Lock trajectory instruction mutex
        trajectory_instruction_buffer_mutex.lock();
        // Try to read all structures from file
        while(file.good())
        {


        }

        //Unlock the mutex when it has finished reading data from the file
        trajectory_instruction_buffer_mutex.unlock();
    }
    else
    {
        std::cout<< "Error while opening file. Exit program"<<std::endl;
        throw std::runtime_error("error");
    }
    file.close();   //Close file after complete read
}

/*
 * Function to communicate with console in automatic mode
 * Displays messages from threads with higher priority
 */
void console_communication_auto_mode(){
    // Read path to predefined trajectory from console
    std::string trajectory_path = read_file_path_from_console();
    // try to open file and read trajectory from file
    read_trajectory_from_file(trajectory_path);
    // display queue data from other thread's

}

/*
 * Function to communicate with console in manual mode
 * Displays messages from threads with higher priority, read console input
 */
void console_communication_manual_mode(){
//TODO: Implement this case



}


/*
 * Function below display welcome message and is waiting for the appropriate mode to be specified
 */
void console_communication_initialization(){
    // Display welcome message
    std::cout <<"Welcome to robot trajectory generator!" <<std::endl;
    // Initialization message queue stack
    pthread_cond_init(&console_communication_stack.empty_buffer, nullptr);
    pthread_cond_init(&console_communication_stack.full_buffer, nullptr);
    pthread_mutex_init(&console_communication_stack.mutex, nullptr);
    console_communication_stack.pointer = 0;
    // Select trajectory generation mode
    select_trajectory_mode();
}

/*
 * Function to communication with console
 * Initialise the data exchange between threads and set the necessary parameters
 */
void console_interface(){
    //Initialize console
    console_communication_initialization();
    // Select mode of communication
    switch(robot_trajectory_mode)
    {
        case Trajectory_control_type::AUTO:     // Automatic mode case
            console_communication_auto_mode();  // Select function to communication in auto mode
            break;
        case Trajectory_control_type::MANUAL:   // Manual mode case
            console_communication_manual_mode();// Select function to communication in manual mode
            break;
        default: //Undefined case
            std::cout<<"Undefined mode! Stop application."<<std::endl; // Display error message
            throw std::runtime_error("error"); //Stop program
    }
    // Display exit message
    std::cout<<"Exit program. Thank you for your root access!"<<std::endl;
}


