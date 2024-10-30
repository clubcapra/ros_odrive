cmake_minimum_required(VERSION 3.8)
project(odrive_joint_broadcaster)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  # rosidl_default_generators
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ODriveJointState.msg"
  "srv/RequestClearErrors.srv"
  "srv/RequestReboot.srv"
)
ament_export_dependencies(rosidl_default_runtime)

# Add odrive_joint_broadcaster library related compile commands
generate_parameter_library(odrive_joint_broadcaster_parameters
src/odrive_joint_broadcaster.yaml
include/odrive_joint_broadcaster/validate_odrive_joint_broadcaster_parameters.hpp
)

include_directories(include)

add_library(
  odrive_joint_broadcaster_
  SHARED
  src/odrive_joint_broadcaster.cpp
  )
  
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_include_directories(odrive_joint_broadcaster_ PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/odrive_joint_broadcaster_parameters/include>"
  "$<INSTALL_INTERFACE:${PROJECT_NAME}_parameters/include>"
  "$<INSTALL_INTERFACE:${PROJECT_NAME}/include>")
target_link_libraries(odrive_joint_broadcaster_ odrive_joint_broadcaster_parameters)
ament_target_dependencies(odrive_joint_broadcaster_ ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(odrive_joint_broadcaster_ PRIVATE "ODRIVE_JOINT_BROADCASTER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface odrive_joint_broadcaster.xml)

install(
  TARGETS
  odrive_joint_broadcaster_
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)


install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  odrive_joint_broadcaster_
)


ament_package()
