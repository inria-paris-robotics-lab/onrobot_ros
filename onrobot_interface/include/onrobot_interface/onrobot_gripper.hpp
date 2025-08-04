#ifndef ONROBOT_GRIPPER_HPP
#define ONROBOT_GRIPPER_HPP

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <thread>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <ur_msgs/srv/set_io.hpp>
#include <ur_msgs/msg/io_states.hpp>
#include <ur_msgs/msg/tool_data_msg.hpp>
#include <std_msgs/msg/string.hpp>

#include <functional>
using namespace std::chrono_literals;



class OnRobotGripper{
    float DEFAULT_MAX_POSITION_VOLTAGE_RG2_RG6 = 3.0; // Default max position voltage for RG2/RG6
    float DEFAULT_MAX_POSITION_VOLTAGE_RG6_V2 = 10.0; // Default max position voltage for RG6 V2
    float DEFAULT_VOLTAGE = 24.0; // Default voltage for the tool
    int DEFAULT_STATE = 0; // Default state of the gripper (0 for open, 1 for closed)
    int PIN_GRIPPER_CONTROL = 16; // Pin used for gripper control
    int PIN_GRIPPER_STATE = 17; // Pin used for gripper state
    public:
        OnRobotGripper(){};
        ~OnRobotGripper(){};
        OnRobotGripper(const rclcpp::Node::SharedPtr& node, const std::string prefix, const std::string model);
        void enable();
        bool disable();
        bool is_enabled() const;
        bool init_communication();

        bool is_busy() const;
        double get_position() const;
        void open(bool low_force_mode=false);
        void close(bool low_force_mode=false);
        void execute_command();

    private:
        // Parameters
        std::atomic<int> _pending_command;
        std::atomic<bool>_low_force_mode; // Low force mode flag
        std::atomic<bool> _command_in_progress;
        std::atomic<double> _current_position; // Current position of the gripper
        std::atomic<double> _last_known_position;
        rclcpp::Time _last_position_change_time;
        rclcpp::Node::SharedPtr _node;
        std::string _prefix;
        std::atomic<int> _target_state; // Target state of the gripper (0 for open, 1 for closed)
        std::atomic<double> _tool_voltage; // Voltage of the tool
        std::atomic<double> _position_voltage; // Voltage of the position sensor
        std::atomic<int> _state; // Current state of the gripper (0 for open, 1 for closed)
        std::string _model; // Model of the gripper (e.g., "rg2", "rg6", "rg6_v2")
        double _max_position_voltage; // Max voltage for the position sensor
        rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr _set_io; // Client to set IOs
        rclcpp::Subscription<ur_msgs::msg::IOStates>::SharedPtr _states_io_sub; // Subscription to IO states
        rclcpp::Subscription<ur_msgs::msg::ToolDataMsg>::SharedPtr _tool_data_sub; // Subscription to tool data

        // Methods
        void _set_tool_voltage(float voltage);
        void _set_digital_output(int fun, int pin, float state);
        void ioStatesCallback(const ur_msgs::msg::IOStates::SharedPtr io_states);
        void toolDataCallback(const ur_msgs::msg::ToolDataMsg::SharedPtr tool_data);
        void _move(int target, bool low_force_mode=false);

};

#endif