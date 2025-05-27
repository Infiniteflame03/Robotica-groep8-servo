cmake_minimum_required(VERSION 3.8)
project(servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(dynamixel_sdk REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Locate wiringPi library
find_library(WIRINGPI_LIB wiringPi)
if(NOT WIRINGPI_LIB)
  message(FATAL_ERROR "wiringPi library not found")
endif()

set(msg_files "msg/Position.msg")
rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files})
ament_export_dependencies(rosidl_default_runtime)

add_executable(main src/main.cpp
        src/PositionNode.cpp
        src/PositionNode.h
        src/ServoControl.cpp
        src/ServoControl.h
        src/AX12A/AX12A.cpp
        src/AX12A/AX12A.h
        src/AX12A/HardwareSerialRPi.cpp
        src/AX12A/HardwareSerialRPi.h)
ament_target_dependencies(main
  rclcpp
  std_msgs
  dynamixel_sdk
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

target_include_directories(main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(main PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++

# Link with wiringPi
target_link_libraries(main ${WIRINGPI_LIB} "${cpp_typesupport_target}")

install(TARGETS main
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
