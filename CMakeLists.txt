cmake_minimum_required(VERSION 3.8)
project(roboteq_motor_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}_msgs
  "msg/RPM.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

add_executable(roboteq_motor_controller src/roboteq_motor_controller.cpp)
target_link_libraries(roboteq_motor_controller ${catkin_LIBRARIES})
ament_target_dependencies(roboteq_motor_controller rclcpp rclpy message_filters)


rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_msgs "rosidl_typesupport_cpp")
target_link_libraries(roboteq_motor_controller "${cpp_typesupport_target}")

install(TARGETS
  roboteq_motor_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
