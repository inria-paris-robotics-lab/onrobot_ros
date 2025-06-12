#include <onrobot_interface/onrobot_interface.hpp>
namespace onrobot_interface
{
    OnRobotHardwareInterface::~OnRobotHardwareInterface() {}

    hardware_interface::CallbackReturn OnRobotHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
    {
        // Valider qu'on a bien un seul joint défini dans l'URDF pour ce matériel
        if (info_.joints.size() != 1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("OnRobotHardwareInterface"), "Un seul joint doit être défini. Trouvé : %zu", info_.joints.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        joint_name_ = info_.joints[0].name;
        prefix_ = info.hardware_parameters.at("prefix");
        // Initialiser les variables d'état et de commande
        hw_position_command_ = 0.0;
        hw_position_state_ = 0.0; 
        hw_velocity_state_ = 0.0;
        hw_effort_state_ = 0.0;

        RCLCPP_INFO(rclcpp::get_logger("OnRobotHardwareInterface"), "Initialisation réussie pour le joint '%s'.", joint_name_.c_str());
        return hardware_interface::CallbackReturn::SUCCESS;
    }

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

    std::vector<hardware_interface::CommandInterface> OnRobotHardwareInterface::export_command_interfaces()
    {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    RCLCPP_INFO(rclcpp::get_logger("OnRobotHardwareInterface"), "Exporting command interface for joint '%s'.", joint_name_.c_str());
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_name_, hardware_interface::HW_IF_POSITION, &hw_position_command_));
    return command_interfaces;
    }

    hardware_interface::CallbackReturn OnRobotHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("OnRobotHardwareInterface"), "Configuration...");
        // Créer le noeud et le driver
        node_ = rclcpp::Node::make_shared("onrobot_gripper_hw_interface_node");
        gripper_ = std::make_unique<OnRobotGripper>(node_, prefix_);
        RCLCPP_INFO(rclcpp::get_logger("OnRobotHardwareInterface"), "Création du driver OnRobotGripper avec le préfixe '%s'.", prefix_.c_str());
        is_active_ = false; // Initialiser le flag d'activation
        stop_thread_ = false; // Initialiser le flag d'arrêt du thread

        node_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        node_executor_->add_node(node_);

        // 3. Lancer un thread qui ne fait que "spinner" cet executor
        node_thread_ = std::thread([this]() {
            RCLCPP_INFO(this->node_->get_logger(), "Démarrage du thread de spin pour l'interface du gripper.");
            while (!is_active_ && !stop_thread_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Attendre que le gripper soit actif
            }
            if (stop_thread_) {
                RCLCPP_INFO(this->node_->get_logger(), "Thread de spin arrêté avant l'activation du gripper.");
                return;
            }
            this->node_executor_->spin();
            RCLCPP_INFO(this->node_->get_logger(), "Arrêt du thread de spin pour l'interface du gripper.");
        });

        RCLCPP_INFO(rclcpp::get_logger("OnRobotHardwareInterface"), "Configuration terminée avec succès.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn OnRobotHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("OnRobotHardwareInterface"), "Activation... Lancement du thread d'initialisation.");
        init_thread_ = std::thread(&OnRobotHardwareInterface::_init_thread_loop, this);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn OnRobotHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("OnRobotHardwareInterface"), "Désactivation...");
        stop_thread_ = true; // Indiquer au thread d'arrêt
        if (init_thread_.joinable()) {
            init_thread_.join(); // Attendre que le thread d'initialisation se termine
        }
        // Désactiver le gripper
        if ( gripper_->is_enabled() ) {
            if (!gripper_->disable()) {
                RCLCPP_ERROR(rclcpp::get_logger("OnRobotHardwareInterface"), "Échec de la désactivation du gripper.");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("OnRobotHardwareInterface"), "Gripper désactivé.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn OnRobotHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("OnRobotHardwareInterface"), "Nettoyage...");

        // <<< AJOUTER CETTE SECTION >>>
        // 1. Dire à l'executor d'arrêter de spinner
        if (node_executor_) {
            node_executor_->cancel();
        }
        // 2. Attendre que le thread se termine
        if (node_thread_.joinable()) {
            node_thread_.join();
        }
        
        // 3. Nettoyer les objets
        gripper_.reset();
        node_executor_.reset();
        node_.reset();

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type OnRobotHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        // Lire l'état du gripper
        hw_position_state_ = gripper_->get_position();
        // RCLCPP_INFO(rclcpp::get_logger("OnRobotHardwareInterface"), "Position lue du gripper: %f", hw_position_state_);
        // Si le driver est en train d'exécuter un mouvement...
        if (gripper_->is_busy())
        {
            // ... on rapporte une vitesse arbitraire non-nulle.
            // La valeur exacte n'a pas d'importance, tant qu'elle est > stalled_velocity_threshold.
            hw_velocity_state_ = 0.1; 
        }
        else
        {
            // Si aucun mouvement n'est en cours, la vitesse est nulle.
            hw_velocity_state_ = 0.0;
        }

        // L'effort est toujours nul.
        hw_effort_state_ = 0.0;
        return hardware_interface::return_type::OK;
    }
    hardware_interface::return_type OnRobotHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        // Écrire la commande dans le gripper
        if (std::abs(hw_position_command_ - hw_position_state_) > 1e-2) {
            if(gripper_->is_busy()) {
                // RCLCPP_WARN(rclcpp::get_logger("OnRobotHardwareInterface"), "Gripper is busy, cannot send new command.");
                // return hardware_interface::return_type::OK;
            } else {
                // RCLCPP_INFO(rclcpp::get_logger("OnRobotHardwareInterface"), "Sending command to gripper: %f", hw_position_command_);
                if (hw_position_command_ > hw_position_state_) {
                    gripper_->close();
                } else {
                    gripper_->open();
                }
            }
        }
        return hardware_interface::return_type::OK;;
    }
    void OnRobotHardwareInterface::_init_thread_loop()
    {
    // 1. Initialiser la communication (boucle jusqu'à succès)
    while (!gripper_->init_communication() && !stop_thread_) {
        RCLCPP_INFO(node_->get_logger(), "En attente du service SetIO... (nouvelle tentative dans 2s)");
        std::this_thread::sleep_for(std::chrono::seconds(2));
    } 
    
    if (stop_thread_) {
        RCLCPP_INFO(node_->get_logger(), "Thread d'initialisation arrêté pendant l'attente du service.");
        return;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Communication établie !");

    // 2. Activer le matériel
    RCLCPP_INFO(node_->get_logger(), "Materiel non activé");
    gripper_->enable();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(node_->get_logger(), "Matériel activé avec succès !");

    // 3. Mettre à jour l'état partagé pour que la boucle de contrôle puisse commencer
    is_active_ = true;
    return;
    }

  #include "pluginlib/class_list_macros.hpp"
  PLUGINLIB_EXPORT_CLASS(onrobot_interface::OnRobotHardwareInterface, hardware_interface::SystemInterface)

} // namespace onrobot_interface