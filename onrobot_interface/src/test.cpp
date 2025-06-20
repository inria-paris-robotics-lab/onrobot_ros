#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "onrobot_interface/onrobot_gripper.hpp" // Assurez-vous que le chemin est correct

using namespace std::chrono_literals;

class GripperTesterNode : public rclcpp::Node
{
public:
    // <<< CHANGEMENT 1 : Le constructeur est maintenant minimaliste.
    GripperTesterNode() : Node("gripper_tester_node")
    {
        RCLCPP_INFO(this->get_logger(), "Constructeur du noeud de test appelé.");
        // ON NE FAIT RIEN D'AUTRE ICI
    }

    // <<< CHANGEMENT 2 : On crée une méthode d'initialisation.
    /**
     * @brief Initialise les composants qui dépendent d'un shared_ptr (comme le driver).
     * Doit être appelée APRES la création du noeud avec std::make_shared.
     */
    void init()
    {
        RCLCPP_INFO(this->get_logger(), "Initialisation du noeud de test...");
    
        try {
            // C'est maintenant sûr d'appeler shared_from_this() ici !
            gripper_driver_ = std::make_unique<OnRobotGripper>(this->shared_from_this(), "right_");
        } catch (const std::runtime_error& e) {
            RCLCPP_FATAL(this->get_logger(), "Échec de l'initialisation du driver: %s", e.what());
            rclcpp::shutdown();
            return;
        }
        
        // La création du timer est aussi déplacée ici par cohérence.
        timer_ = this->create_wall_timer(
            500ms, std::bind(&GripperTesterNode::test_sequence_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Initialisation terminée. La séquence va commencer...");
    }

private:
    void test_sequence_callback()
    {
        // ... (le reste de la fonction est identique)
        if (test_step_ > 8) {
            RCLCPP_INFO(this->get_logger(), "Séquence de test terminée.");
            timer_->cancel();
            rclcpp::shutdown();
            return;
        }

        switch (test_step_)
        {
            case 0:
                RCLCPP_INFO(this->get_logger(), "[Test Step %d] Attente de l'activation... (is_enabled = %s)",
                            test_step_, gripper_driver_->is_enabled() ? "true" : "false");
                if (gripper_driver_->is_enabled()){
                    RCLCPP_INFO(this->get_logger(), "Activation réussie !");
                    std::this_thread::sleep_for(2s); // Pause pour stabiliser
                    test_step_++;
                    break;
                }
                if(!gripper_driver_->is_busy()) {
                    RCLCPP_INFO(this->get_logger(), "[Test Step %d] Commande d'activation...", test_step_);
                    gripper_driver_->enable(); // On active le gripper
                }
                // std::this_thread::sleep_for(2s); // Pause pour stabiliser

                RCLCPP_INFO(this->get_logger(), "Commande d'activation envoyée.");
                break;
            case 1:
                RCLCPP_INFO(this->get_logger(), "[Test Step %d] Commande de FERMETURE...", test_step_);
                gripper_driver_->close();
                test_step_++;
                break;
            case 2:
                RCLCPP_INFO(this->get_logger(), "[Test Step %d] Attente de la fin du mouvement (is_busy = %s)", 
                            test_step_, gripper_driver_->is_busy() ? "true" : "false");
                if (!gripper_driver_->is_busy()) {
                    RCLCPP_INFO(this->get_logger(), "Mouvement de fermeture terminé.");
                    std::this_thread::sleep_for(2s);
                    test_step_++;
                }
                break;
            case 3:
                RCLCPP_INFO(this->get_logger(), "[Test Step %d] Commande d'OUVERTURE...", test_step_);
                gripper_driver_->open();
                test_step_++;
                break;
            case 4:
                RCLCPP_INFO(this->get_logger(), "[Test Step %d] Attente de la fin du mouvement (is_busy = %s)",
                            test_step_, gripper_driver_->is_busy() ? "true" : "false");
                if (!gripper_driver_->is_busy()) {
                    RCLCPP_INFO(this->get_logger(), "Mouvement d'ouverture terminé.");
                    std::this_thread::sleep_for(2s);
                    test_step_++;
                }
                break;
            case 5:
                RCLCPP_INFO(this->get_logger(), "[Test Step %d] Désactivation du gripper...", test_step_);
                gripper_driver_->disable();
                test_step_++;
                break;
            default:
                test_step_++;
                break;
        }
    }

    std::unique_ptr<OnRobotGripper> gripper_driver_;
    rclcpp::TimerBase::SharedPtr timer_;
    int test_step_{0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // <<< CHANGEMENT 3 : La séquence de démarrage est modifiée.
    // 1. On crée le noeud. Le constructeur est appelé ici.
    auto tester_node = std::make_shared<GripperTesterNode>();
    
    // 2. ON APPELLE LA MÉTHODE init() MAINTENANT QUE LE SHARED_PTR EXISTE.
    tester_node->init();
    
    // 3. On lance le spin.
    rclcpp::spin(tester_node);
    
    rclcpp::shutdown();
    return 0;
}