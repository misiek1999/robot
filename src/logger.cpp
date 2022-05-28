//
// Created by Ja on 16.05.2022.
//
#include "logger.h"

// Message queue for data from console to logger
mqd_t	mes_to_logger_queue;
struct	mq_attr mes_to_logger_queue_attr;
log_mes_que_data_t rec_data;    // Buffer for income data

// File with logged data
std::ofstream file_log;

// Return current time in string
std::string get_time_in_string(){
    // read current date
    time_t curr_time;
    tm * curr_tm;
    time(&curr_time);
    //TODO: Isnieje ryzyko że ta funkcja zawiesi się na wielowątkowym wywołaniu bo korzysta ze zmiennych statycznych
    curr_tm = localtime(&curr_time);
    char time_str[32];
    strftime(time_str, 32, "%H:%M:%S -%d.%m.%Y", curr_tm);
    return std::string(time_str);
};

//Initialization of data log
void log_init(){
    // read current date
    time_t curr_time;
    tm * curr_tm;
    time(&curr_time);
    //TODO: Isnieje ryzyko że ta funkcja zawiesi się na wielowątkowym wywołaniu bo korzysta ze zmiennych statycznych
    curr_tm = localtime(&curr_time);
    char file_name[32];
    // Format string from date
    strftime(file_name, 32, "log_%H%M_%d%m%Y.log", curr_tm);
    file_log.open(file_name,std::fstream::out);
    file_log<<"Program launched at: "<<get_time_in_string()<<std::endl;
    if (!file_log.is_open())
        std::cerr << "Failed to open " << file_name << '\n';
}

// Read data from message queue and save it to file
void save_data_to_file(){
    // Read current time
    time_t curr_time;
    tm * curr_tm;
    time(&curr_time);
    curr_tm = localtime(&curr_time);
    // Format string from date
    char string_to_save[64];
    strftime(string_to_save, 32, "[%T] ", curr_tm);
    // Timeout for receive data
    struct timespec rec_timeout = {1, 0};
    // read data from queue
    int status = mq_timedreceive(mes_to_logger_queue, (char *)&rec_data[0], sizeof(log_mes_que_data_t), NULL, &rec_timeout);
    if (status >= 0 ) { // If message is receive successful then save it to file
        // Add two string
        strcat(string_to_save, rec_data);
        // Save data to file
        file_log << string_to_save << std::endl;
    }
}

// Logging data to file from message queue
void * log_data_to_file(void *pVoid){
    // Init thread priority
    int policy;     //Scheduling policy: FIFO or RR
    struct sched_param param;   //Structure of other thread parameters
    /* Read modify and set new thread priority */
    pthread_getschedparam( pthread_self(), &policy, &param);
    param.sched_priority = sched_get_priority_min(policy);  // Read minimum value for thread priority
    pthread_setschedparam( pthread_self(), policy, &param);   //set minimum thread priority for this thread


    // Initialization of logging
    log_init();
    // Enter to loop until the end of the programme
    while (get_program_state() != ControllerState::CLOSE_PROGRAM){
        // Read data from queue and save to file
        save_data_to_file();
    }
    file_log<<"Program stop at: "<<get_time_in_string()<<std::endl;
    // Close file
    file_log.close();
    return 0;
}

// Send message to log queue
void log_string(const  char* str_to_log){
    int status = mq_send(mes_to_logger_queue, (const char *)&str_to_log[0], sizeof(log_mes_que_data_t), 0);
    // Catch error code
    if (status < 0 )
        std::cerr<<"MQ SEND ERROR: "<<status<<" -> "<< strerror(errno) <<std::endl;
}