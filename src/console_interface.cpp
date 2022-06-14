//
// Created by Ja on 18.05.2022.
//


#include "console_interface.h"

/*
 * Define message queue to receive messages from other thread's
 * Read data from other thread and display message in console
 * This structure is default in posix message queue
 */
mqd_t	mes_to_console_queue;
struct	mq_attr mes_to_console_queue_attr;
mq_consol_data_t  mq_console_read_data;

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
        if (get_program_state() == ProgramState::CLOSE_PROGRAM)
            return;
    }
    // Change char number to enum number
    robot_trajectory_mode = static_cast<Trajectory_control_type> (mode - 48);
    // Display correct value
    if (robot_trajectory_mode == Trajectory_control_type::AUTO) {
        std::cout << "Selected  mode: Auto" << std::endl;
        write_to_log("Selected mode : Auto");
    }
    else{
        std::cout << "Selected  mode: Manual"<< std::endl;
        write_to_log("Selected mode : Manual");
    }
}

/*
 * Send csting to console from other threads
 */
void write_to_console(const  char* str_to_console){
    int status = mq_send(mes_to_console_queue, (const char *)&str_to_console[0], sizeof(mq_consol_data_t), 0);
    // Catch error code
    if (status < 0 )
        std::cerr<<"MQ CONSOLE SEND ERROR: "<<status<<" -> "<< strerror(errno) <<std::endl;
}

/*
 * Function read data from queue and display it on console
 * Inline function to reduce unnecessary calls
 */
inline void display_queue_messages(){
    // Timeout for receive data 25ms
    struct timespec rec_timeout = {0, 25000000};
    // read data from queue
    int status = mq_timedreceive(mes_to_console_queue, (char *)&mq_console_read_data[0], sizeof(mq_consol_data_t), NULL, &rec_timeout);
    if (status >= 0 ) { // If message is receive successful then save it to file
        // Print read data in console
        std::cout<< mq_console_read_data << std::endl;
    }
}

/*
 * Message displayed after close program
 */
void close_console(){
//    std::cout <<"Program end."<<std::endl;
}

#define ECHOFLAGS (ECHO | ECHOE | ECHOK | ECHONL)   // Define flags for linux terminal
/*
 * Display control instruction in manual mode in console
 */
void display_manual_instruction(){
    // Display control instruction
    std::cout <<"--------------------------------------"<<std::endl;
    std::cout << "To move 1 joint press:  < [q] - [w] >"<<std::endl;
    std::cout << "To move 2 joint press:  < [a] - [s] >"<<std::endl;
    std::cout << "To move 3 joint press:  < [z] - [x] >"<<std::endl;
    std::cout << "To write binary output press number from 1 to 8"<<std::endl;
    std::cout << "Press 'c' to exit"<<std::endl;
    // Disable console input display
    struct termios termios;
    tcgetattr(STDIN_FILENO, &termios);
    termios.c_lflag &= ~ECHO;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &termios);
    termios .c_lflag &=~ECHOFLAGS;
}

/*
 * Read control from console
 */
void read_control_from_console(){
    // Create temp variable for income char
    char read_char;
    // Set the terminal to raw mode, works only on linux
    system("stty raw");
    read_char = getchar();    // Read char from console
    // Select right option
    switch (read_char) {
        case 'c':
            set_program_state(ProgramState::CLOSE_PROGRAM);  // Close program after press 'q'
            break;
        case 'q':
            send_manual_control(ManualModeControlInstruction::joint_1_left);  // Turn 1 joint
            break;
        case 'w':
            send_manual_control(ManualModeControlInstruction::joint_1_right);  // Turn 1 joint
            break;
        case 'a':
            send_manual_control(ManualModeControlInstruction::joint_2_left);  // Turn 2 joint
            break;
        case 's':
            send_manual_control(ManualModeControlInstruction::joint_2_right);   // Turn 2 joint
            break;
        case 'z':
            send_manual_control(ManualModeControlInstruction::joint_3_left);   // Turn 3 joint
            break;
        case 'x':
            send_manual_control(ManualModeControlInstruction::joint_3_right);   // Turn 3 joint
            break;
        // If input is number from 1 to 8 then change robot digital output
        case 49 ... 56:
            // change number from 1 ... 8 to powers of 2
            set_digital_output((uint8_t) pow(read_char - 48, 2));
            break;
        default:
            break;  // Do nothing after selecting undefined key;
    }
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
            path_is_correct = true; // end loop when file exist
        else
            std::cout<<"Invalid path! Please specified valid path." <<std::endl;
        // If program is shutdown, then return nullptr
        if (get_program_state() == ProgramState::CLOSE_PROGRAM)
            return std::string("");
    }
    // Works only on unix system
    std::cout<<"Specified path is correct. Path: \\e[3 " <<path_to_file<<" \\e[0m" <<std::endl;
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
        // Lock trajectory instruction mutex
        trajectory_instruction_buffer_mutex.lock();
        // Try to read all structures from file
        while(file.good())
        {
            // Read instructions from binary file
            file.read((char*)&trajectory_instruction_buffer, sizeof(trajectory_instruction_buffer));
        }
        //Unlock the mutex when it has finished reading data from the file
        trajectory_instruction_buffer_mutex.unlock();
        is_file_trajectory_load = true;
        // Display message after successful read data from file
        std::cout<< "File loaded successful! "<<std::endl;
        char str[32];
        sprintf(str,"Trajectory loaded from file: %s", _path_to_file.c_str());
        write_to_log(str);
    }
    else
    {
        // Display error message and throw error
        std::cout<< "Error while opening file. Exit program"<<std::endl;
        throw std::runtime_error("error");
    }
    file.close();   //Close file after completed read
}

/*
 * Function to communicate with console in automatic mode
 * Displays messages from threads with higher priority
 */
void console_communication_auto_mode(){
    // Read path to predefined trajectory from console
    std::string trajectory_path = read_file_path_from_console();
    // If program is shutdown before final setup, close function
    if (get_program_state() == ProgramState::CLOSE_PROGRAM)
        return;
    // try to open file and read trajectory from file
    read_trajectory_from_file(trajectory_path);
    // Enter to infinity loop until the program is terminated
    while(get_program_state() != ProgramState::CLOSE_PROGRAM){
        // display queue data from other thread's
        display_queue_messages();
        // Wait 10ms to next
        usleep(10000);
    }
    // Display message after closing console
    close_console();
}

/*
 * Function to communicate with console in manual mode
 * Displays messages from threads with higher priority, read console input
 */
void console_communication_manual_mode(){
    // Display console instruction in manual mode
    display_manual_instruction();
    // Enter to infinity loop until the program is terminated
    while(get_program_state() != ProgramState::CLOSE_PROGRAM){
        // read control from console
        read_control_from_console();
        // display queue data from other thread's
        display_queue_messages();
        //        // Wait 10ms to next iteration
        //        usleep(10000);
    }
    // Display message after closing console
    close_console();
}

/*
 * Function below display welcome message and is waiting for the appropriate mode to be specified
 */
void console_communication_initialization(){
    // Display welcome message
    std::cout <<"Welcome to robot trajectory generator!" <<std::endl;
    // Select trajectory generation mode
    select_trajectory_mode();
}

/*
 * Function to communication with console
 * Initialise the data exchange between threads and set the necessary parameters
 */
void * console_interface(void *pVoid){
    // Init thread priority
    int policy;     //Scheduling policy: FIFO or RR
    struct sched_param param;   //Structure of other thread parameters
    /* Read modify and set new thread priority */
    pthread_getschedparam( pthread_self(), &policy, &param);
    param.sched_priority = sched_get_priority_min(policy);  // Read minimum value for thread priority
    pthread_setschedparam( pthread_self(), policy-1, &param);   //set almost minimum thread priority for this thread

    //Initialize console
    console_communication_initialization();
    if (get_program_state() != ProgramState::CLOSE_PROGRAM)
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
    // Display exit message in console
    std::cout<<"Exit program. Thank you for your root access!"<<std::endl;
    return 0;
}



