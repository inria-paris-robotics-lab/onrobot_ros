#include "onrobot_interface/onrobot_gripper.hpp"


OnRobotGripper::OnRobotGripper(const rclcpp::Node::SharedPtr& node, const std::string prefix, const std::string model){
    this->_node = node;
    this->_prefix = prefix;
    this->_model = model;

    this->_command_in_progress.store(false);
    this->_target_state.store(DEFAULT_STATE); // Target state of the gripper (0 for open, 1 for closed)

    this->_tool_voltage.store(0.0); // Voltage of the tool
    this->_position_voltage.store(0.0);
    this->_state.store(0); // Current state of the gripper (0 for open, 1 for closed)
    if (this->_model == "rg6_v2") {
        this->_max_position_voltage = DEFAULT_MAX_POSITION_VOLTAGE_RG6_V2;
    } else{
        this->_max_position_voltage = DEFAULT_MAX_POSITION_VOLTAGE_RG2_RG6; // Default max position voltage for RG2/RG6
    }


    this->_set_io = this->_node->create_client<ur_msgs::srv::SetIO>("/" + this->_prefix + "io_and_status_controller/set_io");
    this->_states_io_sub = this->_node->create_subscription<ur_msgs::msg::IOStates>(
        "/"+_prefix + "io_and_status_controller/io_states",10,
        std::bind(&OnRobotGripper::ioStatesCallback, this, std::placeholders::_1));
    this->_tool_data_sub = this->_node->create_subscription<ur_msgs::msg::ToolDataMsg>(
        "/"+_prefix + "io_and_status_controller/tool_data", 10,
        std::bind(&OnRobotGripper::toolDataCallback, this, std::placeholders::_1));
    this->_pending_command.store(-1);
    this->_last_known_position.store(0.0);
    this->_last_position_change_time = this->_node->get_clock()->now();

}

/***
 * Initializes the communication with the gripper.
 * This method checks if the SetIO service is ready and waits for it if not.
 * @return true if the service is ready, false otherwise.
 * If the service is not available, it logs an error and returns false.
 * If the service is already ready, it logs an info message and returns true.
 */
bool OnRobotGripper::init_communication() {
    // Si c'est déjà initialisé, ne rien faire
    if (this->_set_io->service_is_ready()) {
        RCLCPP_INFO(this->_node->get_logger(), "Service SetIO déjà prêt.");
        return true;
    }
    // Attendre que le service SetIO soit prêt
    if (!this->_set_io->wait_for_service(1s)) {
        RCLCPP_WARN(this->_node->get_logger(), "Service SetIO non disponible. Impossible d'initialiser le gripper.");
        return false;
    }
    RCLCPP_INFO(this->_node->get_logger(), "Communication du gripper initialisée avec succès.");
    return true;
}

/***
 * Checks if the gripper is busy (processing a command).
 * @return true if the gripper is busy, false otherwise.
 */
bool OnRobotGripper::is_busy() const {
    return this->_command_in_progress.load(); // Check if a command is in progress
}
/***
 * Gets the current position of the gripper.
 * @return the current position of the gripper as a double.
 */
double OnRobotGripper::get_position() const {
    return this->_current_position.load();
}

/***
 * Checks if the gripper is enabled (i.e., if the tool voltage is above a certain threshold).
 * @return true if the gripper is enabled, false otherwise.
 */
bool OnRobotGripper::is_enabled() const {
    // The gripper is considered enabled if the voltage is correct.
    return this->_tool_voltage.load() > 23.0;
}
/***
 * Sets the tool voltage to a specified value.
 * @param voltage the voltage to set for the tool, must be between 0 and 24V.
 * If the voltage is out of range, it logs an error and does not send the command.
 */
void OnRobotGripper::_set_tool_voltage(float voltage) {
    if (voltage < 0.0 || voltage > DEFAULT_VOLTAGE) {
        RCLCPP_ERROR(this->_node->get_logger(), "Tool voltage must be between 0 and 24V");
        return;
    }
    this->_set_digital_output(4, 0, voltage); // the pin is ignored just set fun to 4 to set the voltage
}
/**
 * Sets a digital output for the gripper.
 * @param fun the function to set (1 for digital output, 2 for set flag, 3 for analog output, 4 for tool voltage).
 * @param pin the pin number to set (16 for gripper control, 17 for gripper state).
 * @param state the state to set (0 or 1 for digital output, voltage value for tool voltage).
 * If the SetIO service is not ready, it logs an error and returns.
 */
void OnRobotGripper::_set_digital_output(int fun, int pin, float state) {
    if (!this->_set_io->service_is_ready()) {
        RCLCPP_ERROR(_node->get_logger(), "Unable to send command, SetIO service is not ready.");
        return;
    }
    auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
    request->fun = fun;
    request->pin = pin;
    request->state = int(state);
    this->_set_io->async_send_request(request);
}
/***
 * Callback function for IO states. It updates the gripper's state and ready status based on the IO states received.
 * @param io_states the IO states message containing the digital input states.
 * It checks pin 16 for the gripper state and pin 17 for the ready state.
 * If a command was in progress and the gripper is ready and in the target state, it resets the command in progress flag.
 */
void OnRobotGripper::ioStatesCallback(const ur_msgs::msg::IOStates::SharedPtr io_states){
    for (const auto& io : io_states->digital_in_states) {
        if (io.pin == PIN_GRIPPER_CONTROL) { // Pin 16 is used for the gripper
            this->_state.store(int(io.state)); // Update the ready state based on pin 16
        }
    }
}
/***
 * Callback function for tool data. It updates the tool voltage and position voltage based on the received tool data.
 * It also calculates the current position of the gripper based on the position voltage.
 * @param tool_data the tool data message containing the tool output voltage and analog input values.
 * It scales the position to a range of [0, 1.3] based on the position voltage and max position voltage.
 */
void OnRobotGripper::toolDataCallback(const ur_msgs::msg::ToolDataMsg::SharedPtr tool_data) {
    this->_tool_voltage.store(tool_data->tool_output_voltage); // Update the tool voltage
    this->_position_voltage.store(tool_data->analog_input2); // Update the position voltage
    // Position is calculated based on the position voltage and max position voltage
    if (this->_max_position_voltage > 1e-3) {
        float pourcent_pos = std::max(0.0, std::min(1.0, (this->_position_voltage - 0.6) / (this->_max_position_voltage - 0.6))); // 0.6V = pos 1.3, maxV = pos 0
        pourcent_pos = 1.0 - pourcent_pos; // Inverse pour que 0V=max pos, 0.6V=min pos
        this->_current_position.store(pourcent_pos * 1.3); // Scale the position to [0, 1.3]
        bool movement_finished = false; // Flag to check if the movement is finished
        // Condition 1 target pos reached
        if (this->_command_in_progress.load() && this->_state.load() == this->_target_state.load()) {
            if(std::abs(pourcent_pos - this->_target_state.load()) < 0.02) { // If the gripper is in the target state with a tolerance of 2%
                movement_finished = true;
            }
        }
        // Condition 2 Gripper stalled
        const double SIGNIFICANT_POSITION_CHANGE = 0.01; // Changement of 1%
        const rclcpp::Duration STALL_TIMEOUT = rclcpp::Duration(1, 0); // 5 seconds the gripper should have finished moving in this time

        if (std::abs(this->_current_position.load() - this->_last_known_position.load()) > SIGNIFICANT_POSITION_CHANGE) {
            // The gripper is still moving, update the time and position
            this->_last_position_change_time = this->_node->get_clock()->now();
            this->_last_known_position.store(this->_current_position.load());
        } else {
            // La position n'a pas changé de manière significative. Vérifions si le timeout est dépassé.
            if ((this->_node->get_clock()->now() - this->_last_position_change_time) > STALL_TIMEOUT) {
                movement_finished = true;
            }
        }
        // If the movement is finished, reset the command in progress flag
        this->_command_in_progress.store(!movement_finished);
    }
}
/***
 * Enables the gripper by setting the tool voltage to 24V and initializing the digital output pins.
 * It waits for the gripper to be powered up before proceeding.
 * If the gripper is already enabled, it logs an info message and does not change the state.
 * If the tool voltage is not 24V, it sets it to 24V and toggles pin 16 to wake up the gripper.
 */
void OnRobotGripper::enable() {
    this->_command_in_progress.store(true); // Reset command in progress
    RCLCPP_INFO(this->_node->get_logger(), "Activating OnRobot gripper...");
    this->_set_digital_output(1, PIN_GRIPPER_CONTROL, 0); // Always start with the pin 16 set to Low
    
    if (!this->is_enabled()) {
        printf("Setting tool voltage to 24V...\n");
        this->_set_tool_voltage(DEFAULT_VOLTAGE); // Set the tool voltage
        // Wait for the voltage to stabilize, with a timeout of 5 seconds.
        rclcpp::Time start_time = this->_node->get_clock()->now();
        while (rclcpp::ok() && !this->is_enabled() && (this->_node->get_clock()->now() - start_time) < rclcpp::Duration(5, 0)) {
            RCLCPP_INFO(this->_node->get_logger(), "Waiting for gripper to power up...");
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        std::this_thread::sleep_for(std::chrono::seconds(5)); // Wait for 5 seconds to ensure the gripper is powered up and booted up

    }
    else{
        RCLCPP_INFO(this->_node->get_logger(), "Gripper is already enabled.");
    }

    // Switching the pin 16 from High to Low (after power on) is necessary to 'wake up' the gripper.
    RCLCPP_INFO(this->_node->get_logger(), "Performing gripper wake-up sequence...");
    this->_set_digital_output(1, PIN_GRIPPER_CONTROL, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Wait for 100ms to ensure the gripper is ready 
    this->_set_digital_output(1, PIN_GRIPPER_CONTROL, 0); 
    RCLCPP_INFO(this->_node->get_logger(), "Gripper enabled with tool voltage: %.2fV", this->_tool_voltage.load());

    this->_command_in_progress.store(false); // Reset command in progress after enabling
    RCLCPP_INFO(this->_node->get_logger(), "Gripper is now enabled and ready to operate.");
}
/***
 * Disables the gripper by setting the tool voltage to 0V.
 * It logs an info message indicating that the command to disable the gripper has been sent.
 * @return true if the command was sent successfully, false otherwise.
 */
bool OnRobotGripper::disable() {
    this->_set_tool_voltage(0.0);
    RCLCPP_INFO(this->_node->get_logger(), "Disable command sent.");
    return true;
}
/***
 * Moves the gripper to the target state (open or close).
 * @param target the target state of the gripper (0 for open, 1 for close).
 * @param low_force_mode if true, uses low force mode for the gripper.
 * If a command is already in progress, it logs a warning and returns without moving.
 * It sets the target state and starts the movement by setting the digital outputs accordingly.
 */
void OnRobotGripper::_move(int target, bool low_force_mode) {
    if (this->_command_in_progress.load()) {
        // A movement is already in progress, do nothing.
        RCLCPP_WARN(_node->get_logger(), "MOVE: Command already in progress, ignoring new command.");
        return;
    }

    RCLCPP_INFO(this->_node->get_logger(), "MOVE: Starting movement to target: %d", target);
    this->_command_in_progress.store(true); // Lock to prevent new commands
    this->_target_state.store(target);

    this->_last_position_change_time = this->_node->get_clock()->now();
    this->_last_known_position.store(this->_current_position.load());

    // If the gripper is already in the target state AND it is ready,
    // there is no reason to send another command.
    if (this->_state.load() == target) {
        // The gripper is already in the desired state.
        return;
    }

    

    // These commands will be sent only once at the beginning of the movement.
    this->_set_digital_output(1, PIN_GRIPPER_STATE, low_force_mode ? 1 : 0);
    this->_set_digital_output(1, PIN_GRIPPER_CONTROL, target);
}
/**
 * Opens the gripper by moving it to the open state (target 0).
 * @param low_force_mode if true, uses low force mode for the gripper.
 * It calls the _move method with target 0 to open the gripper.
 */
void OnRobotGripper::open(bool low_force_mode) {
    this->_pending_command.store(0);
    this->_low_force_mode.store(low_force_mode);
}
/***
 * Closes the gripper by moving it to the closed state (target 1).
 * @param low_force_mode if true, uses low force mode for the gripper.
 * It calls the _move method with target 1 to close the gripper.
 */
void OnRobotGripper::close(bool low_force_mode) {
    this->_pending_command.store(1);
    this->_low_force_mode.store(low_force_mode);
}

void OnRobotGripper::execute_command()
{
    // Load the pending command atomically
    int command = _pending_command.load();

    // If no command is pending, return immediately
    if (command == -1) {
        return;
    }

    // Retrieve the command and reset the mailbox
    // The atomic exchange ensures that we only process the same command once
    command = _pending_command.exchange(-1);
    if (command == -1) return; // Another thread may have taken it just before

    RCLCPP_INFO(_node->get_logger(), "Executing command: %d", command);

    // Execute the movement
    if (command == 1) { // 1 = close
        this->_move(1,  this->_low_force_mode.load());
    } else { // 0 = open
        this->_move(0, this->_low_force_mode.load());
    }
}
