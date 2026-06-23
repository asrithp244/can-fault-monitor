cmake_minimum_required(VERSION 3.16)
project(can_fault_monitor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake       REQUIRED)
find_package(rclcpp            REQUIRED)
find_package(rclcpp_lifecycle  REQUIRED)
find_package(std_msgs          REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CanFrame.msg"
  "msg/CanHealthStatus.msg"
  "msg/FaultEvent.msg"
  "srv/InjectFault.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

add_library(can_socket STATIC src/can_socket.cpp)
target_include_directories(can_socket PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(can_socket rclcpp)

macro(add_can_node target source)
  add_executable(${target} ${source})
  target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(${target} can_socket)
  ament_target_dependencies(${target} rclcpp rclcpp_lifecycle std_msgs)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(${target} "${cpp_typesupport_target}")
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endmacro()

add_can_node(can_publisher      src/can_publisher_node.cpp)
add_can_node(can_subscriber     src/can_subscriber_node.cpp)
add_can_node(fault_injector     src/fault_injector_node.cpp)
add_can_node(can_health_monitor src/can_health_monitor_node.cpp)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch   DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config   DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts  DESTINATION share/${PROJECT_NAME} USE_SOURCE_PERMISSIONS)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
