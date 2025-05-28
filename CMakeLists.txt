cmake_minimum_required(VERSION 3.8)
project(mdriver)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
include_directories(include)

set(msg_files
  "msg/Status.msg"
)

set(srv_files
  "srv/StateTransition.srv")

add_executable(mdriver_node src/mdriver_node.cpp)
add_executable(mdriver_node_test src/mdriver_node_test.cpp)
ament_target_dependencies(mdriver_node rclcpp std_msgs)
ament_target_dependencies(mdriver_node_test rclcpp std_msgs)

rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files} ${srv_files}
    DEPENDENCIES std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(mdriver_node "${cpp_typesupport_target}")
target_link_libraries(mdriver_node_test "${cpp_typesupport_target}")

install(TARGETS
  mdriver_node
  mdriver_node_test
  DESTINATION lib/${PROJECT_NAME})

#install launch files
install(DIRECTORY ${mdriver_SOURCE_DIR}/launch/
  DESTINATION share/${PROJECT_NAME})

#note: had to add '_py' suffix to the python source and install directory to prevent `ament_python_install_package()` from failing
#see also https://github.com/ament/ament_cmake/issues/514
ament_python_install_package(${PROJECT_NAME}_py)

#note: had to add '_py' suffix to the python source and install directory to prevent `ament_python_install_package()` from failing
#see also https://github.com/ament/ament_cmake/issues/514
install(PROGRAMS
  mdriver_py/mdriver_node.py
  mdriver_py/gui.py
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

ament_package()

file(GLOB python_scripts scripts/*.py)