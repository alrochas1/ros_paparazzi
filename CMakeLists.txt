cmake_minimum_required(VERSION 3.8)
project(ros_paparazzi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# maybe i can delete the cpp parts and just use python?
# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(cpp_executable src/cpp_node.cpp)
ament_target_dependencies(cpp_executable rclcpp)
# Install Cpp executables
install(TARGETS
  cpp_executable
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules (here you can create your executables)
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/raspy
  scripts/computer
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
