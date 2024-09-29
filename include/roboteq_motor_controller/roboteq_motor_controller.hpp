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
    /**
     * @class Roboteq Motor Controller class. 
     */
    class MotorController : public rclcpp::Node
    {
        private:
            /**
             * @brief Motor rpm command subscriber to give RPM command left and right motors.
             *        This subscriber subscribes /roboteq_motor_controller/motors_rpm topic and gets left and right motor rpm. 
             *        Check motor_controller::msg::RPM message type. 
             */
            rclcpp::Subscription<roboteq_motor_controller::msg::RPM>::SharedPtr motor_controller_rpm_command_subscriber_;

            /**
             * @brief Motors rpm publisher with RPM type. Please check roboteq_motor_controller::msg::RPM message type. 
             *        This publisher publishes left and right motors rpm to /roboteq_motor_controller/motors_rpm_output topic.
             */
            rclcpp::Publisher<roboteq_motor_controller::msg::RPM>::SharedPtr motor_controller_rpm_publisher_;

            /**
             * @brief Motor controller flags publisher. Please check roboteq_motor_controller::msg::MotorControllerFlags message type.
             *        This publisher publishes motor controller flags to /roboteq_motor_controller/motor_controller_flags topic. 
             */
            rclcpp::Publisher<roboteq_motor_controller::msg::MotorControllerFlags>::SharedPtr motor_controller_flags_publisher_;

            /**
             * @brief Motor controller status publisher. Please check roboteq_motor_controller::msg::MotorControllerStatus message type.
             *        This publisher publishes motor controller status to /roboteq_motor_controller/motor_controller_status topic.
             */
            rclcpp::Publisher<roboteq_motor_controller::msg::MotorControllerStatus>::SharedPtr motor_controller_status_publisher_;

            /**
             * @brief Motor controller read timer.
             */
            rclcpp::TimerBase::SharedPtr motor_controller_read_timer_; 
            
            /**
             * @brief Motor controller write timer.
             */
            rclcpp::TimerBase::SharedPtr motor_controller_write_timer_;

            /**
             * @brief RPM message which is getted from motor controller.
             */
            roboteq_motor_controller::msg::RPM rpm_message_from_driver_;
            
            /**
             * @brief RPM message which will be sended to motor controller.
             */
            roboteq_motor_controller::msg::RPM rpm_message_to_driver_;

            /**
             * @brief Motor controller flags for publishing.
             */
            roboteq_motor_controller::msg::MotorControllerFlags motor_controller_flags_message_;

            /**
             * @brief Motor controller status for publishing.
             */
            roboteq_motor_controller::msg::MotorControllerStatus motor_controller_status_message_;

            /**
             * @brief Motor controller status flags with binary array. It will give binary array like ["0", "1", "1", "0"] and each bit gives information about motor controller flags.
             */
            int motor_controller_status_flags_binary_array_[MOTOR_CONTROLLER_STATUS_ARRAY_LENGTH];

            /**
             * @brief Left motor flags binary array.
             */
            int left_motor_flags_binary_array_[MOTOR_FLAGS_ARRAY_LENGTH];

            /**
             * @brief Right motor flags binary array.
             */
            int right_motor_flags_binary_array_[MOTOR_FLAGS_ARRAY_LENGTH];   

            /**
             * @brief Motor controller fault falgs binary array.
             */
            int motor_controller_fault_flags_binary_array_[MOTOR_CONTROLLER_FAULT_FLAGS_ARRAY_LENGTH];     

            /**
             * @brief Status flags.
             */
            const std::string status_flags_[MOTOR_CONTROLLER_STATUS_ARRAY_LENGTH];
            
            /**
             * @brief Motor flags.
             */
            const std::string motor_flags_[MOTOR_FLAGS_ARRAY_LENGTH];

            /**
             * @brief Fault flags.
             */
            const std::string fault_flags_[MOTOR_CONTROLLER_FAULT_FLAGS_ARRAY_LENGTH];

            /**
             * @brief Motor controller TTY (termios) for connecting motor controller via USB-serial.
             */
            struct termios motor_controller_tty_;

            /**
             * @brief Motor controller serial port.
             */
            int motor_controller_serial_port_;

            /**
             * @brief Motor controller serial port name. (/dev/ttyACM0, /dev/ttyUSB0 etc.)
             */
            std::string motor_controller_serial_port_name_;

            /**
             * @brief Motor controller baudrate.
             */
            int motor_controller_baudrate_;

            /**
             * @brief Roboteq motor controller frame id for std_msgs::Header.
             */
            std::string roboteq_motor_controller_frame_id_;

            /**
             * @brief Motor controller read buffer which is readed from serial port.
             */
            char motor_controller_read_buffer_[MOTOR_CONTROLLER_BUFFER_LENGTH];

            /**
             * @brief Left and right motor directions. These values must be 1 or -1.
             */
            int left_motor_direction_, right_motor_direction_;

            /**
             * @brief Motor controller regex iterator start and end.
             */
            std::string::const_iterator motor_controller_regex_iterator_start_, motor_controller_regex_iterator_end_;

            /**
             * @brief Combined patthern for parsing motor and motor controller values from readed buffer.
             */
            std::regex combined_pattern_;

            /**
             * @brief Matches values which are parsed via regex.
             */
            std::smatch matches_;

            /**
             * @brief Motor controller endline command.
             */
            std::string endline_command_ = "\r\n";

            /**
             * @brief Clear motor controller buffer command. 
             */
            const std::string clear_motor_controller_buffer_command_;

            /**
             * @brief Speed query command for left and right motors.
             */
            const std::string speed_query_command_;

            /**
             * @brief Motor controller current query command.
             */
            const std::string current_query_command_;

            /**
             * @brief Motor controller temperature query command.
             */
            const std::string temperature_query_command_;

            /**
             * @brief Motor controller state flag query command.
             */
            const std::string state_flag_command_;

            /**
             * @brief Motors flag query command.
             */
            const std::string motors_flag_command_;

            /**
             * @brief Motors fault flag query command.
             */
            const std::string fault_flag_command_;

            /**
             * @brief Read period command. This works with 1/T logic. 
             */
            const std::string read_period_command_;
            
            /**
             * @brief Acceleration command for left and right motors.
             */
            const std::string acceleration_command_;

            /**
             * @brief Deceleration command for left and right motors.
             */
            const std::string deceleration_command_;

            /**
             * @brief Speed command for left and right motors. RPM values obtained from the RPM subscriber will be sent to the motor controller using this command.
             */
            const std::string speed_command_;

            /**
             * @brief Write period command. This works with 1/T logic.
             */
            const std::string write_period_command_;
            
            /**
             * @brief Motor controller readed bytes. 
             */
            int motor_controller_readed_bytes_;

            /**
             * @brief Motor controller readed data which is converted from bytes.
             */
            std::string motor_controller_readed_data_;

            /**
             * @brief Parsed serial data via regex.
             */
            std::string motor_controller_serial_data_;

            /**
             * @brief Motor acceleration value.
             */
            int motor_acceleration_;

            /**
             * @brief Motor deceleration value.
             */
            int motor_deceleration_;

        protected:

        public:
            /**
             * @brief Constructor of MotorController class.
             */
            MotorController();

            /**
             * @brief Destructor of MotorController class.
             */
            ~MotorController();

            /**
             * @brief Motor RPM Callback function for write RPM values to motor controller.
             * @param message (const roboteq_motor_controller::msg::RPM::SharedPtr) RPM message.
             */
            void motorRPMCallback(const roboteq_motor_controller::msg::RPM::SharedPtr message);
            
            /**
             * @brief Setter baudrate of serial port function.
             */
            void settingBaudrate(void);

            /**
             * @brief Function for setting termios of roboteq motor controller.
             */
            void prepareTermios(void);

            /**
             * @brief Function to clear motor controller buffer.
             */
            void clearMotorControllerBuffer(void);

            /**
             * @brief Setter motors and motor controller values. This function initializes variables of motors and motor controllers via serial.
             */
            void setMotorsAndMotorControllerValues(void);

            /**
             * @brief Reader motors and motor controller values function. rclcpp::Timer uses this function (motor_controller_read_timer_).
             */
            void readMotorsAndMotorControllerValues(void);

            /**
             * @brief Writer motors values function. rclcpp::Timer uses this function (motor_controller_write_timer_). 
             */
            void writeMotorsValues(void);

            /**
             * @brief Parser function for motors and motor controller variables via regex.
             */
            void parseMotorsAndMotorControllerData(void);

            /**
             * @brief Converter decimal to binary array digits function. This function is used for getting motor and motor controller flags.
             * @param decimal_value (unsigned int) Decimal value which is converted to binary array.
             * @param bit_count (int) Array length
             * @param binary_array (int *) Binary array with flags.
             */
            void convertDecimalToBinaryArrayOfDigits(unsigned int decimal_value, int bit_count, int *binarry_array);
    };
} // namespace Roboteq



#endif // ROBOTEQ_MOTOR_CONTROLLER_HPP_
