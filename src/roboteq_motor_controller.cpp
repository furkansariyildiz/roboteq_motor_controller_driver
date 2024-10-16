#include "roboteq_motor_controller/roboteq_motor_controller.hpp"


Roboteq::MotorController::MotorController()
    : Node("roboteq_motor_controller_node"), 
      status_flags_{"Setup", "RunScript", "STO", "AtLimit", "StallDetected", "PowerStageOff", "AnalogMode", "PulseMode", "SerialMode"},
      motor_flags_{"AmpsTriggerActivated", "ReverseLimitTriggered", "ForwardLimitTriggered", "SafetyStopActive", "LoopErrorDetected", "MotorStalled", "AmpsLimitCurrentlyActive"},
      fault_flags_{"DefaultConfigLoaded", "MosfetFail", "MotorSensorSetupFault", "EmergencyStop", "ShotCircuit", "UnderVoltage", "OverVoltage", "OverHeat"},
      clear_motor_controller_buffer_command_(std::string("# C\r\n") + std::string("#\r\n")),
      speed_query_command_(std::string("?S") + endline_command_),
      current_query_command_(std::string("?A") + endline_command_),
      temperature_query_command_(std::string("?T") + endline_command_),
      state_flag_command_(std::string("?FS") + endline_command_),
      motors_flag_command_(std::string("?FM") + endline_command_),
      fault_flag_command_(std::string("?FF") + endline_command_),
      read_period_command_(std::string("# 1.0") + endline_command_),
      acceleration_command_("!AC"),
      deceleration_command_("!DC"),
      speed_command_("!S"),
      write_period_command_("!MS"),
      motor_controller_readed_bytes_(0),
      motor_controller_readed_data_(""),
      motor_controller_serial_data_("") {
    // Subscribers
    motor_controller_rpm_command_subscriber_ = this->create_subscription<roboteq_motor_controller::msg::RPM>("/roboteq_motor_controller/motors_rpm", 1000, std::bind(&MotorController::motorRPMCallback, this, std::placeholders::_1));

    // Publishers
    motor_controller_rpm_publisher_ = this->create_publisher<roboteq_motor_controller::msg::RPM>("/roboteq_motor_controller/motors_rpm_output", 10);
    motor_controller_flags_publisher_ = this->create_publisher<roboteq_motor_controller::msg::MotorControllerFlags>("/roboteq_motor_controller/motor_controller_flags", 10);
    motor_controller_status_publisher_ = this->create_publisher<roboteq_motor_controller::msg::MotorControllerStatus>("/roboteq_motor_controller/motor_controller_status", 10);

    // Timers
    motor_controller_read_timer_ = this->create_wall_timer(std::chrono::milliseconds(1), bind(&MotorController::readMotorsAndMotorControllerValues, this));
    motor_controller_write_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), bind(&MotorController::writeMotorsValues, this));

    // Parameters
    declare_parameter("roboteq_motor_controller.port_name", "/dev/ttyUSB0");
    declare_parameter("roboteq_motor_controller.frame_id", "roboteq_motor_controller");
    declare_parameter("roboteq_motor_controller.baudrate", 115200);
    declare_parameter("roboteq_motor_controller.left_motor_direction", 1);
    declare_parameter("roboteq_motor_controller.right_motor_direction", 1);
    declare_parameter("roboteq_motor_controller.motor_acceleration", 1.0);
    declare_parameter("roboteq_motor_controller.motor_deceleration", 1.0);

    motor_controller_serial_port_name_ = this->get_parameter("roboteq_motor_controller.port_name").as_string();
    roboteq_motor_controller_frame_id_ = this->get_parameter("roboteq_motor_controller.frame_id").as_string();
    motor_controller_baudrate_ = this->get_parameter("roboteq_motor_controller.baudrate").as_int();
    left_motor_direction_ = this->get_parameter("roboteq_motor_controller.left_motor_direction").as_int();
    right_motor_direction_ = this->get_parameter("roboteq_motor_controller.right_motor_direction").as_int();
    motor_acceleration_ = this->get_parameter("roboteq_motor_controller.motor_acceleration").as_double();
    motor_deceleration_ = this->get_parameter("roboteq_motor_controller.motor_deceleration").as_double();

    this->settingBaudrate();

    motor_controller_serial_port_ = open(motor_controller_serial_port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    
    // Regex patterns initialize
    combined_pattern_ = std::regex("(S=(-?\\d+):(-?\\d+))|(A=(-?\\d+):(-?\\d+))|(T=(-?\\d+):(-?\\d+):(-?\\d+))|(FS=(\\d+))|(FM=(\\d+):(\\d+))|(FF=(\\d+))");

    this->prepareTermios();
    this->clearMotorControllerBuffer();
    this->setMotorsAndMotorControllerValues();

    // System sleep duration
    rclcpp::sleep_for(std::chrono::milliseconds(2000));

    RCLCPP_INFO_STREAM(this->get_logger(), "Port name: " << motor_controller_serial_port_name_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Frame ID: " << roboteq_motor_controller_frame_id_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Baudrate: " << motor_controller_baudrate_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Left Motor Direction: " << left_motor_direction_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Right Motor Direction: " << right_motor_direction_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Motor Acceleration: " << motor_acceleration_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Motor Deceleration: " << motor_deceleration_);
}



Roboteq::MotorController::~MotorController() {
    close(motor_controller_serial_port_);
}



void Roboteq::MotorController::motorRPMCallback(const roboteq_motor_controller::msg::RPM::SharedPtr message) {
    rpm_message_to_driver_ = *message;
}



void Roboteq::MotorController::settingBaudrate() {
    switch (motor_controller_baudrate_) {
    case 9600:
        motor_controller_baudrate_ = B9600;
        break;
    
    case 19200:
        motor_controller_baudrate_ = B19200;
        break;

    case 38400:
        motor_controller_baudrate_ = B38400;
        break;

    case 57600:
        motor_controller_baudrate_ = B57600;
        break;

    case 115200:
        motor_controller_baudrate_ = B115200;
        break;

    default:
        motor_controller_baudrate_ = B9600;
        break;
    }
}



void Roboteq::MotorController::prepareTermios() {
    cfsetospeed(&motor_controller_tty_, (speed_t)motor_controller_baudrate_);
    cfsetispeed(&motor_controller_tty_, (speed_t)motor_controller_baudrate_);

    motor_controller_tty_.c_cflag     &=  ~PARENB;
    motor_controller_tty_.c_cflag     &=  ~CSTOPB;
    motor_controller_tty_.c_cflag     &=  ~CSIZE;
    motor_controller_tty_.c_cflag     |=  CS8;

    motor_controller_tty_.c_cflag     &=  ~CRTSCTS;           
    motor_controller_tty_.c_cc[VMIN]   =  1;                  
    motor_controller_tty_.c_cc[VTIME]  =  5;                  
    motor_controller_tty_.c_cflag     |=  CREAD | CLOCAL;  

    cfmakeraw(&motor_controller_tty_);   
    tcflush(motor_controller_serial_port_, TCIFLUSH);

    if(tcsetattr(motor_controller_serial_port_, TCSANOW, &motor_controller_tty_) != 0) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Can not configure roboteq motor controller serial port!");
        rclcpp::shutdown();
    }

    RCLCPP_INFO_STREAM(this->get_logger(),  "Roboteq motor controller serial port configuration is completed.");
}



void Roboteq::MotorController::clearMotorControllerBuffer() {
    write(motor_controller_serial_port_, clear_motor_controller_buffer_command_.c_str(), clear_motor_controller_buffer_command_.length());
}



void Roboteq::MotorController::setMotorsAndMotorControllerValues() {
    write(motor_controller_serial_port_, speed_query_command_.c_str(), speed_query_command_.length());
    write(motor_controller_serial_port_, current_query_command_.c_str(), speed_query_command_.length());
    write(motor_controller_serial_port_, temperature_query_command_.c_str(), temperature_query_command_.length());
    write(motor_controller_serial_port_, state_flag_command_.c_str(), state_flag_command_.length());
    write(motor_controller_serial_port_, motors_flag_command_.c_str(), motors_flag_command_.length());
    write(motor_controller_serial_port_, fault_flag_command_.c_str(), fault_flag_command_.length());
    write(motor_controller_serial_port_, read_period_command_.c_str(), read_period_command_.length());
}



void Roboteq::MotorController::readMotorsAndMotorControllerValues() {
    motor_controller_readed_bytes_ = read(motor_controller_serial_port_, motor_controller_read_buffer_, MOTOR_CONTROLLER_BUFFER_LENGTH);

    if(motor_controller_readed_bytes_ > 0) {
        motor_controller_readed_data_ += std::string(motor_controller_read_buffer_, motor_controller_readed_bytes_);
        if(motor_controller_readed_data_.size() >= MOTOR_CONTROLLER_DATA_LIMIT) {
            motor_controller_serial_data_ = std::regex_replace(motor_controller_readed_data_, std::regex(" "), "");
            this->parseMotorsAndMotorControllerData();
            motor_controller_readed_data_ = "";
        }
    } else if(motor_controller_readed_bytes_ == 0) {
        RCLCPP_WARN_STREAM(this->get_logger(), "No data read from the motor controller.");
    } else {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Error reading from the motor controller.");
    }
}



void Roboteq::MotorController::writeMotorsValues() {
    write(motor_controller_serial_port_, (acceleration_command_ + " 1 " + std::to_string(motor_acceleration_) + endline_command_).c_str(), (acceleration_command_ + " 1 " + std::to_string(motor_acceleration_) + endline_command_).size());
    write(motor_controller_serial_port_, (acceleration_command_ + " 2 " + std::to_string(motor_acceleration_) + endline_command_).c_str(), (acceleration_command_ + " 1 " + std::to_string(motor_acceleration_) + endline_command_).size());

    write(motor_controller_serial_port_, (deceleration_command_ + " 1 " + std::to_string(motor_deceleration_) + endline_command_).c_str(), (deceleration_command_ + " 1 " + std::to_string(motor_deceleration_) + endline_command_).size());
    write(motor_controller_serial_port_, (deceleration_command_ + " 2 " + std::to_string(motor_deceleration_) + endline_command_).c_str(), (deceleration_command_ + " 1 " + std::to_string(motor_deceleration_) + endline_command_).size());

    write(motor_controller_serial_port_, (speed_command_ + " 1 " + std::to_string(rpm_message_to_driver_.left_motor_rpm.data) + endline_command_).c_str(), (speed_command_ + " 1 " + std::to_string(rpm_message_to_driver_.left_motor_rpm.data) + endline_command_).size());
    write(motor_controller_serial_port_, (speed_command_ + " 1 " + std::to_string(rpm_message_to_driver_.right_motor_rpm.data) + endline_command_).c_str(), (speed_command_ + " 2 " + std::to_string(rpm_message_to_driver_.right_motor_rpm.data) + endline_command_).size());
}



void Roboteq::MotorController::parseMotorsAndMotorControllerData() {
    motor_controller_regex_iterator_start_ = motor_controller_serial_data_.cbegin();
    motor_controller_regex_iterator_end_ = motor_controller_serial_data_.cend();
    
    rpm_message_from_driver_.header.stamp = this->get_clock()->now();
    rpm_message_from_driver_.header.frame_id = roboteq_motor_controller_frame_id_;

    motor_controller_status_message_.header.stamp = this->get_clock()->now();
    motor_controller_status_message_.header.frame_id = roboteq_motor_controller_frame_id_;

    motor_controller_flags_message_.header.stamp = this->get_clock()->now();
    motor_controller_flags_message_.header.frame_id = roboteq_motor_controller_frame_id_;

    while(std::regex_search(motor_controller_regex_iterator_start_, motor_controller_regex_iterator_end_, matches_, combined_pattern_)) {
        if(matches_[1].matched) {
            // Motors RPM
            rpm_message_from_driver_.left_motor_rpm.data = std::stoi(matches_[2].str());
            rpm_message_from_driver_.right_motor_rpm.data = std::stoi(matches_[3].str());
        } else if(matches_[4].matched) {
            // Motor controller current (A)
            motor_controller_status_message_.left_motor_status.current.amper.data = std::stoi(matches_[5].str()) / 10.0;
            motor_controller_status_message_.right_motor_status.current.amper.data = std::stoi(matches_[6].str()) / 10.0;

        } else if(matches_[7].matched) {
            // Motor controller temperature (C)
            motor_controller_status_message_.left_motor_status.temperature.celsius.data = std::stoi(matches_[8].str());
            motor_controller_status_message_.right_motor_status.temperature.celsius.data = std::stoi(matches_[9].str());

        } else if(matches_[11].matched) {
            // Motor controller status flags 
            convertDecimalToBinaryArrayOfDigits(std::stoi(matches_[12].str()), MOTOR_CONTROLLER_STATUS_ARRAY_LENGTH, motor_controller_status_flags_binary_array_);

            for(int i=0; i<MOTOR_CONTROLLER_STATUS_ARRAY_LENGTH; i++) {
                if(motor_controller_status_flags_binary_array_[i] == 1) {
                    motor_controller_flags_message_.status_flags.push_back(status_flags_[i]);
                }
            }
        } else if(matches_[13].matched) {
            // Motors flags 
            convertDecimalToBinaryArrayOfDigits(std::stoi(matches_[14].str()), MOTOR_FLAGS_ARRAY_LENGTH, left_motor_flags_binary_array_);
            convertDecimalToBinaryArrayOfDigits(std::stoi(matches_[15].str()), MOTOR_FLAGS_ARRAY_LENGTH, right_motor_flags_binary_array_);

            for(int i=0; i<MOTOR_FLAGS_ARRAY_LENGTH; i++) {
                if(left_motor_flags_binary_array_[i] == 1) {
                    motor_controller_flags_message_.left_motor_flags.push_back(motor_flags_[i]);
                }

                if(right_motor_flags_binary_array_[i] == 1) {
                    motor_controller_flags_message_.right_motor_flags.push_back(motor_flags_[i]);
                }
            }
        } else if(matches_[16].matched) {
            // Motor controller fault flags 
            convertDecimalToBinaryArrayOfDigits(std::stoi(matches_[17].str()), MOTOR_CONTROLLER_FAULT_FLAGS_ARRAY_LENGTH, motor_controller_fault_flags_binary_array_);

            for(int i=0; i<MOTOR_CONTROLLER_FAULT_FLAGS_ARRAY_LENGTH; i++) {
                if(motor_controller_fault_flags_binary_array_[i] == 1){
                    motor_controller_flags_message_.fault_flags.push_back(fault_flags_[i]);
                }
            }
        }

        motor_controller_regex_iterator_start_ = matches_[0].second;  
    }

    motor_controller_rpm_publisher_->publish(rpm_message_from_driver_);
    motor_controller_flags_publisher_->publish(motor_controller_flags_message_);
    motor_controller_status_publisher_->publish(motor_controller_status_message_);
    
    motor_controller_flags_message_.status_flags.clear();
    motor_controller_flags_message_.left_motor_flags.clear();
    motor_controller_flags_message_.right_motor_flags.clear();
    motor_controller_flags_message_.fault_flags.clear();
}



void Roboteq::MotorController::convertDecimalToBinaryArrayOfDigits(unsigned int decimal_value, int bit_count, int *binary_array) {
    unsigned int mask = 1U << (bit_count - 1);
    for(int i=0; i<bit_count; i++)
    {
        binary_array[i] = (decimal_value & mask) ? 1 : 0;
        decimal_value <<= 1;
    } 
}



int main(int argc, char *argv[])  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Roboteq::MotorController>());
    return 0;
}


