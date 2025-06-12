#include "onrobot_interface/onrobot_gripper.hpp"


OnRobotGripper::OnRobotGripper(const rclcpp::Node::SharedPtr& node, const std::string prefix){
    this->_node = node;
    this->_prefix = prefix;

    this->_command_in_progress = false;
    this->_current_position = 0.0; // Current position of the gripper
    this->_target_state = 0; // Target state of the gripper (0 for open, 1 for closed)

    this->_tool_voltage = 0.0; // Voltage of the tool
    this->_position_voltage = 0.;
    this->_ready = false;
    this->_state = 0; // Current state of the gripper (0 for open, 1 for closed)
    this->_max_position_voltage = 10.0;  // Max voltage for the position sensor pour le rg2 !!!!!!


    this->_set_io = this->_node->create_client<ur_msgs::srv::SetIO>("/" + this->_prefix + "io_and_status_controller/set_io");
    // RCLCPP_INFO(this->_node->get_logger(), "Création du client SetIO pour le gripper OnRobot:"+"/" + this->_prefix + "io_and_status_controller/set_io");
    // if (!this->_set_io->wait_for_service(5s)) {
    //     RCLCPP_FATAL(this->_node->get_logger(), "Service SetIO non disponible. Impossible d'initialiser le gripper.");
    //     throw std::runtime_error("SetIO service not available");
    // }
    this->_states_io_sub = this->_node->create_subscription<ur_msgs::msg::IOStates>(
        "/"+_prefix + "io_and_status_controller/io_states",10,
        std::bind(&OnRobotGripper::ioStatesCallback, this, std::placeholders::_1));
    this->_tool_data_sub = this->_node->create_subscription<ur_msgs::msg::ToolDataMsg>(
        "/"+_prefix + "io_and_status_controller/tool_data", 10,
        std::bind(&OnRobotGripper::toolDataCallback, this, std::placeholders::_1));
    this->_script_command_pub = this->_node->create_publisher<std_msgs::msg::String>("/"+_prefix + "urscript_interface/script_command", 10);

}

bool OnRobotGripper::init_communication() {
    // Si c'est déjà initialisé, ne rien faire
    if (this->_set_io->service_is_ready()) {
        RCLCPP_INFO(this->_node->get_logger(), "Service SetIO déjà prêt.");
        return true;
    }
    // Attendre que le service SetIO soit prêt
    if (!this->_set_io->wait_for_service(1s)) {
        RCLCPP_FATAL(this->_node->get_logger(), "Service SetIO non disponible. Impossible d'initialiser le gripper.");
        return false;
    }
    RCLCPP_INFO(this->_node->get_logger(), "Communication du gripper initialisée avec succès.");
    return true;
}

bool OnRobotGripper::is_busy() const {
    return this->_command_in_progress;
}

double OnRobotGripper::get_position() const {
    return this->_current_position;
}

bool OnRobotGripper::isReady() {
    return _ready;
}

bool OnRobotGripper::is_enabled() const {
    // Le gripper est considéré comme activé si la tension est correcte.
    return this->_tool_voltage > 23.0;
}

void OnRobotGripper::_set_tool_voltage(float voltage) {
    if (voltage < 0.0 || voltage > 24.0) {
        RCLCPP_ERROR(this->_node->get_logger(), "Tool voltage must be between 0 and 24V");
        return;
    }
    this->_set_digital_output(4, 0, voltage); // the pin is ignored just set fun to 4 to set the voltage
}

void OnRobotGripper::_set_digital_output(int fun, int pin, float state) {
    if (!this->_set_io->service_is_ready()) {
        RCLCPP_ERROR(_node->get_logger(), "Impossible d'envoyer la commande, le service SetIO n'est pas prêt.");
        return;
    }
    auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
    request->fun = fun;
    request->pin = pin;
    request->state = state;
    this->_set_io->async_send_request(request);
}

void OnRobotGripper::ioStatesCallback(const ur_msgs::msg::IOStates::SharedPtr io_states){
    for (const auto& io : io_states->digital_in_states) {
        if (io.pin == 16) { // Pin 16 is used for the gripper
            this->_state = int(io.state); // Update the ready state based on pin 16
        }
        else if (io.pin == 17) { // Pin 17 is used for determine the gripper state
            this->_ready = int(io.state); // Update the gripper state based on pin 17
        }
    }

    // Logique de fin de mouvement : si une commande était en cours et que le gripper
    // signale qu'il est prêt et dans l'état cible, alors le mouvement est terminé.
    if (this->_command_in_progress && this->_ready && this->_state == this->_target_state) {
        // RCLCPP_INFO(_node->get_logger(), "Mouvement du gripper terminé.");
        this->_command_in_progress = false; // On peut accepter une nouvelle commande
    }
}

void OnRobotGripper::toolDataCallback(const ur_msgs::msg::ToolDataMsg::SharedPtr tool_data) {
    this->_tool_voltage = tool_data->tool_output_voltage; // Update the tool voltage
    this->_position_voltage = tool_data->analog_input2; // Update the position voltage
    // Maj de la pos 
    if (this->_max_position_voltage > 1e-3) {
        float pourcent_pos = std::max(0.0, std::min(1.0, (this->_position_voltage - 0.6) / (this->_max_position_voltage - 0.6))); // 0.6V = pos 1.3, maxV = pos 0
        pourcent_pos = 1.0 - pourcent_pos; // Inverse pour que 0V=max pos, 0.6V=min pos
        this->_current_position = pourcent_pos * 1.3; // Scale the position to [0, 1.3]
    }
}

void OnRobotGripper::enable() {
    this->_command_in_progress = true; // Reset command in progress
    RCLCPP_INFO(this->_node->get_logger(), "Activation du gripper OnRobot...");
    this->_set_digital_output(1, 16, 0); // Always start with the pin 16 set to Low
    if (this->_tool_voltage == 24.0 && this->_ready) {
        RCLCPP_INFO(this->_node->get_logger(), "Gripper is already enabled.");
        this->_command_in_progress = false; // Reset command in progress
    }
    else{
        printf("Setting tool voltage to 24V...\n");
        this->_set_tool_voltage(24.0); // Set the tool voltage to 10V
        std::this_thread::sleep_for(std::chrono::seconds(2)); // Wait for 100ms to ensure the gripper is ready 
        // Switching the pin 16 from High to Low (after power on) is necessary to 'wake up' the RG6-V2 gripper.
        this->_set_digital_output(1, 16, 1.);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait for 100ms to ensure the gripper is ready 
        this->_set_digital_output(1, 16, 0.); 
        RCLCPP_INFO(this->_node->get_logger(), "Gripper enabled with tool voltage: %.2fV", this->_tool_voltage);
        this->_command_in_progress = false; // Reset command in progress
    }
}

bool OnRobotGripper::disable() {
    this->_set_tool_voltage(0.0);
    RCLCPP_INFO(this->_node->get_logger(), "Commande de désactivation envoyée.");
    return true;
}

void OnRobotGripper::_move(int target, bool low_force_mode){ // target is 0 for open, 1 for close
    if (this->_command_in_progress) {
        RCLCPP_WARN(_node->get_logger(), "Impossible de bouger, un mouvement est déjà en cours.");
        return;
    }
    // RCLCPP_INFO(_node->get_logger(), "Lancement du mouvement du gripper vers la cible : %d", target);
    this->_command_in_progress = true; // Verrouille pour de nouvelles commandes
    this->_target_state = target;
    this->_ready = false;

    this->_set_digital_output(1, 17, low_force_mode ? 1 : 0);
    this->_set_digital_output(1, 16, target);
}

void OnRobotGripper::open(bool low_force_mode) {
    this->_move(0, low_force_mode); // target 0 for open
}

void OnRobotGripper::close(bool low_force_mode) {
    this->_move(1, low_force_mode); // target 1 for close
}