#ifndef ROBOTEQ_MOTOR_CONTROLLER_HPP_
#define ROBOTEQ_MOTOR_CONTROLLER_HPP_


#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <regex>
#include <iostream>



namespace Roboteq
{
    class MotorController : public rclcpp::Node
    {
        private:
            
        protected:

        public:
            MotorController();

            ~MotorController();
    };
}



#endif
