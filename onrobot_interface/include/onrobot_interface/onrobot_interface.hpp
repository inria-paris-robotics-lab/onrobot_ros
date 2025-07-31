#ifndef ONROBOT_INTERFACE_HPP
#define ONROBOT_INTERFACE_HPP
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include <class_loader/class_loader.hpp>

#include "onrobot_interface/onrobot_gripper.hpp"
#include <memory>
#include <string>

#include <thread>
#include <atomic>

namespace onrobot_interface
{
    class OnRobotHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        OnRobotHardwareInterface() = default;
        ~OnRobotHardwareInterface() override;

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override ;

        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:
        std::unique_ptr<OnRobotGripper> gripper_;
        std::string joint_name_;
        std::string prefix_;
        std::string model_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::TimerBase::SharedPtr command_timer_; // Timer to execute commands periodically
        double hw_position_command_prev_ = 0.0; 
        double hw_position_command_ = 0.0; // Commanded position
        double hw_position_state_ = 0.0; // Current position state
        double hw_velocity_state_ = 0.0; // Current velocity state
        double hw_effort_state_ = 0.0; // Current effort state
        std::thread init_thread_;
        std::atomic<bool> is_initialized_{false}; // Flag to check if the interface is initialized
        std::atomic<bool> stop_thread_{false}; // Flag to check if the interface is configured

        std::thread node_thread_;
        std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> node_executor_;


        void _init_thread_loop();
    };
}

#endif  // ONROBOT_INTERFACE_HPP