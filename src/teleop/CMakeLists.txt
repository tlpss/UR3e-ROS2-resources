cmake_minimum_required(VERSION 3.5)
project(teleop)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package( rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# macro for dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  sensor_msgs
  control_msgs
  moveit_msgs
  geometry_msgs
)

## copy  config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME} )

## copy  launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME} )

# Add executable for using a controller (Component)
add_library(joystick_input SHARED src/ur3e_joystick.cpp)
ament_target_dependencies(joystick_input ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(joystick_input "teleop::JoyToServoPub")

# Install Binaries (for Components)
install(
  TARGETS
  joystick_input
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


ament_package()
