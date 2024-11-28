#include <cstdio>
#include <memory>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

#include "rosgraph_msgs/msg/clock.hpp"
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>

using std::placeholders::_1;

class LatencyEvaluator : public rclcpp::Node
{
    public:
        LatencyEvaluator() : Node("latency_evaluator"){

            velocity_sub_period_flag = true;
            steering_sub_period_flag = true;

            clock_sub_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            velocity_sub_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            steering_sub_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            clock_sub_opt.callback_group = clock_sub_cb_group_;
            velocity_sub_opt.callback_group = velocity_sub_cb_group_;
            steering_sub_opt.callback_group = steering_sub_cb_group_;

            clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>("/clock", 1, std::bind(&LatencyEvaluator::clock_sub_callback, this, _1), clock_sub_opt);

            velocity_status_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", 
                                                                                                    1, std::bind(&LatencyEvaluator::velocity_status_sub_callback, this, _1), 
                                                                                                    velocity_sub_opt);

            steering_status_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", 
                                                                                                    1, std::bind(&LatencyEvaluator::steering_status_sub_callback, this, _1), 
                                                                                                    steering_sub_opt);

            register_count_ = 0;

            auto t = std::time(nullptr);
            auto tm = *std::localtime(&t);

            std::stringstream ss;

            ss << std::put_time(&tm, "%d%m%Y_%H%M%S");

            std::string date_and_time_ = ss.str();

            fout_.open("./src/microAutoware-Latency-Evaluator/data/"+date_and_time_+"_latencies.csv", std::ios::out | std::ios::app);

            fout_ << "#" << ","
                  << "Latency steering" << ","
                  << "Period steering" << ","
                  << "Latency velocity" << ","
                  << "Period velocity"
                  << "\n";  
        
        }

        ~LatencyEvaluator(){
            fout_.close();
        }

    private:

        void clock_sub_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg) {
            clock_ = msg->clock;
        }

        void velocity_status_sub_callback(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg) {
            uint64_t duration = clock_.seconds()*1e3 + clock_.nanoseconds()*1e-9 - msg->header.stamp.sec*1e3 - msg->header.stamp.nanosec*1e-9;
            uint64_t period = clock_.seconds()*1e3 + clock_.nanoseconds()*1e-9 - last_Velocity_msg_time_.seconds()*1e3 - last_Velocity_msg_time_.nanoseconds()*1e-9;
            last_Velocity_msg_time_ = clock_;

            if(velocity_sub_period_flag){
                period = -1;
                velocity_sub_period_flag = false;
            }

            fout_ << ++register_count_ << ","
                  << "-1" << "," 
                  << "-1" << ","
                  << duration << ","
                  << period
                  << "\n";  
        }

        void steering_status_sub_callback(const autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr msg) {
            uint64_t duration = clock_.seconds()*1e3 + clock_.nanoseconds()*1e-9 - msg->stamp.sec*1e3 - msg->stamp.nanosec*1e-9;
            uint64_t period = clock_.seconds()*1e3 + clock_.nanoseconds()*1e-9 - last_Steering_msg_time_.seconds()*1e3 - last_Steering_msg_time_.nanoseconds()*1e-9;
            last_Steering_msg_time_ = clock_;

            if(steering_sub_period_flag){
                period = -1;
                steering_sub_period_flag = false;
            }

            fout_ << ++register_count_ << ","
                  << duration << ","
                  << period << ","
                  << "-1" << "," 
                  << "-1"
                  << "\n";  
                  
        }

        rclcpp::CallbackGroup::SharedPtr clock_sub_cb_group_;
        rclcpp::CallbackGroup::SharedPtr velocity_sub_cb_group_;
        rclcpp::CallbackGroup::SharedPtr steering_sub_cb_group_;

        rclcpp::SubscriptionOptions clock_sub_opt;
        rclcpp::SubscriptionOptions velocity_sub_opt;
        rclcpp::SubscriptionOptions steering_sub_opt;

        rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
        rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_status_sub_;
        rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_status_sub_;

        rclcpp::Time clock_;
        rclcpp::Time last_Steering_msg_time_;
        rclcpp::Time last_Velocity_msg_time_;
        std::string date_and_time_;
        u_int32_t register_count_;

        bool velocity_sub_period_flag;
        bool steering_sub_period_flag;

        std::fstream fout_;

};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    rclcpp::executors::MultiThreadedExecutor executor;

    auto latency_evaluator_node = std::make_shared<LatencyEvaluator>();

    executor.add_node(latency_evaluator_node);

    RCLCPP_INFO(latency_evaluator_node->get_logger(), "Starting latency evaluator node, shut down with CTRL-C");

    executor.spin();

    RCLCPP_INFO(latency_evaluator_node->get_logger(), "Keyboard interrupt, shutting down.\n");

    rclcpp::shutdown();
}
