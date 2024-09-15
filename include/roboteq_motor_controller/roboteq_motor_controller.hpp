#ifndef ROBOTEQ_MOTOR_CONTROLLER_HPP_
#define ROBOTEQ_MOTOR_CONTROLLER_HPP_


#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <chrono>
#include <regex>
#include <iostream>
#include <string.h>
#include <sstream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <features.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

// Messages
#include <roboteq_motor_controller/msg/rpm.hpp>
#include <roboteq_motor_controller/msg/motor_controller_flags.hpp>
#include <roboteq_motor_controller/msg/motor_controller_status.hpp>


#define MOTOR_CONTROLLER_BUFFER_LENGTH                  256
#define MOTOR_CONTROLLER_DATA_LIMIT                     500
#define MOTOR_CONTROLLER_STATUS_ARRAY_LENGTH            9
#define MOTOR_FLAGS_ARRAY_LENGTH                  7
#define MOTOR_CONTROLLER_FAULT_FLAGS_ARRAY_LENGTH       8



namespace Roboteq
{
    class MotorController : public rclcpp::Node
    {
        private:
            /**
             * @brief
             */
            rclcpp::Subscription<roboteq_motor_controller::msg::RPM>::SharedPtr motor_controller_rpm_command_subscriber_;

            /**
             * @brief
             */
            rclcpp::Publisher<roboteq_motor_controller::msg::RPM>::SharedPtr motor_controller_rpm_publisher_;

            rclcpp::Publisher<roboteq_motor_controller::msg::MotorControllerFlags>::SharedPtr motor_controller_flags_publisher_;

            rclcpp::Publisher<roboteq_motor_controller::msg::MotorControllerStatus>::SharedPtr motor_controller_status_publisher_;

            rclcpp::TimerBase::SharedPtr motor_controller_read_timer_; 
            
            rclcpp::TimerBase::SharedPtr motor_controller_write_timer_;

            roboteq_motor_controller::msg::RPM rpm_message_from_driver_;
            
            roboteq_motor_controller::msg::RPM rpm_message_to_driver_;

            roboteq_motor_controller::msg::MotorControllerFlags motor_controller_flags_message_;

            roboteq_motor_controller::msg::MotorControllerStatus motor_controller_status_message_;

            int motor_controller_status_flags_binary_array_[MOTOR_CONTROLLER_STATUS_ARRAY_LENGTH];

            int left_motor_flags_binary_array_[MOTOR_FLAGS_ARRAY_LENGTH];

            int right_motor_flags_binary_array_[MOTOR_FLAGS_ARRAY_LENGTH];   

            int motor_controller_fault_flags_binary_array_[MOTOR_CONTROLLER_FAULT_FLAGS_ARRAY_LENGTH];     

            const std::string status_flags_[MOTOR_CONTROLLER_STATUS_ARRAY_LENGTH];
            
            const std::string motor_flags_[MOTOR_FLAGS_ARRAY_LENGTH];

            const std::string fault_flags_[MOTOR_CONTROLLER_FAULT_FLAGS_ARRAY_LENGTH];

            struct termios motor_controller_tty_;

            int motor_controller_serial_port_;

            std::string motor_controller_serial_port_name_;

            int motor_controller_baudrate_;

            std::string roboteq_motor_controller_frame_id_;

            char motor_controller_read_buffer_[MOTOR_CONTROLLER_BUFFER_LENGTH];

            int left_motor_direction_, right_motor_direction_;

            std::string::const_iterator motor_controller_regex_iterator_start_, motor_controller_regex_iterator_end_;

            std::regex combined_pattern_;
            std::smatch matches_;

            std::regex motors_rpm_pattern_;
            std::smatch motors_rpm_matches_;

            std::regex motor_controller_current_pattern_;
            std::smatch motor_controller_current_matches_;

            std::regex motor_controller_temperature_pattern_;
            std::smatch motor_controller_temperature_matches_;

            std::regex motor_controller_status_flag_pattern_;
            std::smatch motor_controller_status_flag_matches_;

            std::regex motors_flag_pattern_;
            std::smatch motors_flag_matches_;

            std::regex motor_controller_fault_flag_pattern_;
            std::smatch motor_controller_fault_flag_matches_;

            std::string endline_command_ = "\r\n";

            std::string clear_motor_controller_buffer_command_;
            std::string speed_query_command_;
            std::string current_query_command_;
            std::string temperature_query_command_;
            std::string state_flag_command_;
            std::string motors_flag_command_;
            std::string fault_flag_command_;
            std::string read_period_command_;

            std::string acceleration_command_;
            std::string deceleration_command_;
            std::string speed_command_;
            std::string write_period_command_;
            
            int motor_controller_readed_bytes_;
            std::string motor_controller_readed_data_;
            std::string motor_controller_serial_data_;

            int motor_acceleration_;
            int motor_deceleration_;

        protected:

        public:
            MotorController();

            ~MotorController();

            void motorRPMCallback(const roboteq_motor_controller::msg::RPM::SharedPtr message);
            
            void settingBaudrate(void);

            void prepareTermios(void);

            void clearMotorControllerBuffer(void);

            void setMotorsAndMotorControllerValues(void);

            void readMotorsAndMotorControllerValues(void);

            void writeMotorsValues(void);

            void parseMotorsAndMotorControllerData(void);

            void convertDecimalToBinaryArrayOfDigits(unsigned int decimal_value, int bit_count, int *binarry_array);
    };
}



#endif
