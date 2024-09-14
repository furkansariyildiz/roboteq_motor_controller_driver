#include "roboteq_motor_controller/roboteq_motor_controller.hpp"


Roboteq::MotorController::MotorController()
    : Node("roboteq_motor_controller_node") {

}



Roboteq::MotorController::~MotorController() {
    
}



int main(int argc, char *argv[])  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Roboteq::MotorController>());
    return 0;
}


