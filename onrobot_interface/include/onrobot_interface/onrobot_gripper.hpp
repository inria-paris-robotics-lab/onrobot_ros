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
    public:
        OnRobotGripper(){};
        ~OnRobotGripper(){};
        OnRobotGripper(const rclcpp::Node::SharedPtr& node, const std::string prefix);
        void enable();
        bool disable();
        bool isReady();
        bool is_enabled() const;
        bool init_communication();

        bool is_busy() const;
        double get_position() const;
        void open(bool low_force_mode=false);
        void close(bool low_force_mode=false);

    private:
        // Parameters
        bool _command_in_progress;
        double _current_position; // Current position of the gripper
        rclcpp::Node::SharedPtr _node;
        std::string _prefix;
        int _target_state; // Target state of the gripper (0 for open, 1 for closed)
        float _tool_voltage; // Voltage of the tool
        float _position_voltage; // Voltage of the position sensor
        bool _ready; // True if the gripper is ready to operate
        int _state; // Current state of the gripper (0 for open, 1 for closed)
        double _max_position_voltage; // Max voltage for the position sensor
        rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr _set_io; // Client to set IOs
        rclcpp::Subscription<ur_msgs::msg::IOStates>::SharedPtr _states_io_sub; // Subscription to IO states
        rclcpp::Subscription<ur_msgs::msg::ToolDataMsg>::SharedPtr _tool_data_sub; // Subscription to tool data
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _script_command_pub; // Publisher for script commands

        // Methods
        void _set_tool_voltage(float voltage);
        void _set_digital_output(int fun, int pin, float state);
        void ioStatesCallback(const ur_msgs::msg::IOStates::SharedPtr io_states);
        void toolDataCallback(const ur_msgs::msg::ToolDataMsg::SharedPtr tool_data);
        void _move(int target, bool low_force_mode=false);

};

#endif