#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <string>

struct clock_msg{
    u_int32_t sec;
    u_int32_t nanosec;
};

int main(){

    // file pointer
    std::fstream fout;

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::stringstream ss;

    ss << std::put_time(&tm, "%d%m%Y_%H%M%S");

    std::string date_and_time_ = ss.str();

    clock_msg clock_;
    clock_msg last_Steering_msg_time_;
    clock_msg last_Velocity_msg_time_;

    clock_.nanosec = 217575766;
    clock_.sec = 325;

    last_Velocity_msg_time_.nanosec = 117575765;
    last_Velocity_msg_time_.sec = 325;

    last_Steering_msg_time_.nanosec = 17575764;
    last_Steering_msg_time_.sec = 325;

    fout.open(date_and_time_+"_latencies.csv", std::ios::out | std::ios::app);

    fout << "#" << ","
                    << "Latency steering" << ","
                    << "Period steering" << ","
                    << "Latency velocity" << ","
                    << "Period velocity"
                    << "\n";  

    uint64_t duration_velocity = clock_.sec*1.0e9 + clock_.nanosec - last_Steering_msg_time_.sec*1.0e9 - last_Steering_msg_time_.nanosec;
    long double period_velocity = clock_.sec + clock_.nanosec*1.0e-9 - last_Velocity_msg_time_.sec - last_Velocity_msg_time_.nanosec*1.0e-9;
    last_Velocity_msg_time_ = clock_;

    fout << 1 << ","
        << "-1" << "," 
        << "-1" << ","
        << duration_velocity << ","
        << period_velocity
        << "\n";               

}