#include <onrobot_interface/onrobot_interface.hpp>
namespace onrobot_interface
{
    OnRobotHardwareInterface::~OnRobotHardwareInterface() {}
    /***
     * Initialize the hardware interface.
     * @param info the hardware information.
     * @return CallbackReturn::SUCCESS if initialization is successful, ERROR otherwise.
     */
    hardware_interface::CallbackReturn OnRobotHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
    {
        // Validate that only one joint is defined in the URDF for this hardware
        if (info_.joints.size() != 1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("OnRobotHardwareInterface"), "Un seul joint doit être défini. Trouvé : %zu", info_.joints.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        joint_name_ = info_.joints[0].name;
        prefix_ = info.hardware_parameters.at("prefix");
        model_ = info.hardware_parameters.at("model");
        // Initialize state and command variables
        hw_position_command_prev_ = 0.0;
        hw_position_command_ = 0.0;
        hw_position_state_ = 0.0; 
        hw_velocity_state_ = 0.0;
        hw_effort_state_ = 0.0;

        RCLCPP_INFO(rclcpp::get_logger("OnRobotHardwareInterface"), "Initialisation réussie pour le joint '%s'.", joint_name_.c_str());
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    /***
     * Export the state interfaces for the hardware interface.
     * @return a vector of StateInterface objects representing the state interfaces.
     */
    std::vector<hardware_interface::StateInterface> OnRobotHardwareInterface::export_state_interfaces()
    {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_name_, hardware_interface::HW_IF_POSITION, &hw_position_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_name_, hardware_interface::HW_IF_VELOCITY, &hw_velocity_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_name_, hardware_interface::HW_IF_EFFORT, &hw_effort_state_));
    return state_interfaces;
    }
    /***
     * Export the command interfaces for the hardware interface.
     * @return a vector of CommandInterface objects representing the command interfaces.
     */
    std::vector<hardware_interface::CommandInterface> OnRobotHardwareInterface::export_command_interfaces()
    {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    RCLCPP_INFO(rclcpp::get_logger("OnRobotHardwareInterface"), "Exporting command interface for joint '%s'.", joint_name_.c_str());
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_name_, hardware_interface::HW_IF_POSITION, &hw_position_command_));
    return command_interfaces;
    }
    /***
     * Configure the hardware interface.
     * This method initializes the node and the gripper driver, and starts a thread to spin the node executor.
     * @param state the lifecycle state.
     * @return CallbackReturn::SUCCESS if configuration is successful, ERROR otherwise.
     */
    hardware_interface::CallbackReturn OnRobotHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("OnRobotHardwareInterface"), "Configuration...");
        // Create the node and the driver
        node_ = rclcpp::Node::make_shared(prefix_ + "onrobot_gripper_hw_interface_node");
        gripper_ = std::make_unique<OnRobotGripper>(node_, prefix_, model_);
        RCLCPP_INFO(rclcpp::get_logger("OnRobotHardwareInterface"), "Création du driver OnRobotGripper avec le préfixe '%s'.", prefix_.c_str());
        stop_thread_ = false; // Init stop thread flag
        is_initialized_ = false; // Set initialized flag

        node_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        node_executor_->add_node(node_);

        // Start a thread that only "spins" this executor to allow the gripper node to process callbacks.
        node_thread_ = std::thread([this]() {
            RCLCPP_INFO(this->node_->get_logger(), "Starting spin thread for the gripper interface.");
            this->node_executor_->spin();
            RCLCPP_INFO(this->node_->get_logger(), "Stopping spin thread for the gripper interface.");
        });
        command_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), [this]() { this->gripper_->execute_command(); });

        RCLCPP_INFO(rclcpp::get_logger("OnRobotHardwareInterface"), "Configuration completed successfully.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    /***
     * Activate the hardware interface.
     * This method starts a thread to initialize the gripper communication and enable it.
     * @param state the lifecycle state.
     * @return CallbackReturn::SUCCESS if activation is successful, ERROR otherwise.
     */
    hardware_interface::CallbackReturn OnRobotHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("OnRobotHardwareInterface"), "Activating... Starting initialization thread.");
        init_thread_ = std::thread(&OnRobotHardwareInterface::_init_thread_loop, this);
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    /***
     * Deactivate the hardware interface.
     * This method stops the initialization thread and disables the gripper.
     * @param state the lifecycle state.
     * @return CallbackReturn::SUCCESS if deactivation is successful, ERROR otherwise.
     */
    hardware_interface::CallbackReturn OnRobotHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("OnRobotHardwareInterface"), "Deactivating...");
        stop_thread_ = true; // Indicate to stop thread
        if (init_thread_.joinable()) {
            init_thread_.join(); // Wait for the initialization thread to finish
        }
        // Disable the gripper
        if ( gripper_->is_enabled() ) {
            if (!gripper_->disable()) {
                RCLCPP_ERROR(rclcpp::get_logger("OnRobotHardwareInterface"), "Failed to disable gripper.");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("OnRobotHardwareInterface"), "Gripper disabled.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    /***
     * Clean up the hardware interface.
     * This method stops the executor, waits for the thread to finish, and cleans up the objects.
     * @param state the lifecycle state.
     * @return CallbackReturn::SUCCESS if cleanup is successful, ERROR otherwise.
    */
    hardware_interface::CallbackReturn OnRobotHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("OnRobotHardwareInterface"), "Cleaning up...");

        // Tell the executor to stop spinning
        if (node_executor_) {
            node_executor_->cancel();
        }
        // Wait for the thread to finish
        if (node_thread_.joinable()) {
            node_thread_.join();
        }
        // Clean up the objects
        gripper_.reset();
        node_executor_.reset();
        node_.reset();

        return hardware_interface::CallbackReturn::SUCCESS;
    }
    /***
     * Read the state of the hardware interface.
     * This method reads the position, velocity, and effort states of the gripper.
     * @param time the current time.
     * @param period the time period since the last read.
     * @return hardware_interface::return_type::OK if read is successful, ERROR otherwise.
     */
    hardware_interface::return_type OnRobotHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        // Read the state of the gripper
        hw_position_state_ = gripper_->get_position();
        // RCLCPP_INFO(rclcpp::get_logger("OnRobotHardwareInterface"), "Position lue du gripper: %f", hw_position_state_);
        // If the driver is executing a movement...
        if (gripper_->is_busy())
        {
            // we report a non-zero arbitrary speed.
            // The exact value doesn't matter, as long as it's > stalled_velocity_threshold.
            hw_velocity_state_ = 0.1; 
        }
        else
        {
            // If no movement is in progress, the speed is zero.
            hw_velocity_state_ = 0.0;
        }

        // Effort is always zero for this gripper, as it does not report effort.
        hw_effort_state_ = 0.0;
        return hardware_interface::return_type::OK;
    }
    /***
     * Write the command to the hardware interface.
     * This method sends the command to the gripper if it is not busy and the command differs from the current state.
     * @param time the current time.
     * @param period the time period since the last write.
     * @return hardware_interface::return_type::OK if write is successful, ERROR otherwise.
     */
    hardware_interface::return_type OnRobotHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {

        if (!is_initialized_ || gripper_->is_busy())
        {
            return hardware_interface::return_type::OK;
        }
        if (std::abs(hw_position_command_ - hw_position_command_prev_) < 1e-2)
        {
            return hardware_interface::return_type::OK;
        }

        if (hw_position_command_ > hw_position_state_)
        {
            gripper_->close();
        }
        else
        {
            gripper_->open();
        }
        hw_position_command_prev_ = hw_position_command_;

        return hardware_interface::return_type::OK;
    }
    /***
     * The thread loop for initializing the gripper communication and enabling it.
     * This method runs in a separate thread and waits until the communication is established.
     * It then enables the gripper and updates the shared state 
    */
    void OnRobotHardwareInterface::_init_thread_loop()
    {
        // Initialize communication (loop until successful)
        while (!gripper_->init_communication() && !stop_thread_) {
            RCLCPP_INFO(node_->get_logger(), "Waiting for SetIO service... (retrying in 2s)");
            std::this_thread::sleep_for(std::chrono::seconds(2));
        } 
        
        if (stop_thread_) {
            RCLCPP_INFO(node_->get_logger(), "Initialization thread stopped while waiting for the service.");
            return;
        }
        
        RCLCPP_INFO(node_->get_logger(), "Communication established!");

        // Activate the hardware
        RCLCPP_INFO(node_->get_logger(), "Hardware not activated");
        std::this_thread::sleep_for(std::chrono::seconds(2));
        gripper_->enable();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        RCLCPP_INFO(node_->get_logger(), "Hardware successfully activated!");
        is_initialized_ = true; 
        RCLCPP_INFO(node_->get_logger(), "Initialization complete. Control loop is now unlocked.");
        return;
    }

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(onrobot_interface::OnRobotHardwareInterface, hardware_interface::SystemInterface)

} // namespace onrobot_interface