cmake_minimum_required(VERSION 3.8)
project(roboteq_motor_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(message_filters REQUIRED)
# find_package(serial REQUIRED)

include_directories(include)

add_executable(roboteq_motor_controller src/roboteq_motor_controller.cpp)
target_link_libraries(roboteq_motor_controller ${catkin_LIBRARIES})
ament_target_dependencies(roboteq_motor_controller rclcpp rclpy message_filters)

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
