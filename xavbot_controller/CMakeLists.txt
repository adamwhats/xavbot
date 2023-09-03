cmake_minimum_required(VERSION 3.8)
project(xavbot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## DEPENDENCIES
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  Eigen3
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  xavbot_controller
  SHARED
  src/mecanum_drive_controller.cpp
)

target_include_directories(
    xavbot_controller
    PRIVATE
    "include"
)

target_link_libraries(xavbot_controller PUBLIC serial)

ament_target_dependencies(
  xavbot_controller PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

## INSTALL
install(
        TARGETS xavbot_controller
        DESTINATION lib
)
install(
        DIRECTORY include/
        DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

## EXPORTS
pluginlib_export_plugin_description_file(controller_interface mecanum_drive_controller.xml)

ament_export_include_directories(
  include
)
ament_export_libraries(
  franka_hardware
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  )
ament_package()
