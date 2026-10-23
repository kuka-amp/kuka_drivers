cmake_minimum_required(VERSION 3.5)
project(kuka_rsi_driver)

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

find_package(ament_cmake REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(kuka_driver_interfaces REQUIRED)
find_package(kuka-external-control-sdk-iiqka2 CONFIG REQUIRED)
find_package(kuka_drivers_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

# Create the library
add_library(
  ${PROJECT_NAME} SHARED
  src/hardware_interface_rsi_base.cpp
  src/rsi_xml_configuration_parser.cpp
  src/hardware_interface_eki_rsi.cpp
  src/hardware_interface_rsi_only.cpp
  src/hardware_interface_mxa_rsi.cpp
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "KUKA_RSI_DRIVER_BUILDING_LIBRARY")
# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

target_link_libraries(${PROJECT_NAME}
  hardware_interface::hardware_interface
  kuka_drivers_core::kuka_drivers_core
  Kuka::kuka-external-control-sdk-iiqka2
  yaml-cpp)

# Shared library for robot_manager_base
add_library(robot_manager_base STATIC src/robot_manager_base.cpp)
target_link_libraries(robot_manager_base
  std_msgs::std_msgs
  kuka_driver_interfaces::kuka_driver_interfaces
  kuka_drivers_core::kuka_drivers_core
  controller_manager_msgs::controller_manager_msgs
  kuka_drivers_core::communication_helpers)

add_executable(robot_manager_node_extended src/robot_manager_node_extended.cpp)
target_link_libraries(robot_manager_node_extended PRIVATE robot_manager_base)

add_executable(robot_manager_node_rsi_only src/robot_manager_node_rsi_only.cpp)
target_link_libraries(robot_manager_node_rsi_only PRIVATE robot_manager_base)

pluginlib_export_plugin_description_file(hardware_interface eki_rsi_hardware_interface.xml)
pluginlib_export_plugin_description_file(hardware_interface rsi_hardware_interface.xml)
pluginlib_export_plugin_description_file(hardware_interface mxa_rsi_hardware_interface.xml)


install(
  TARGETS ${PROJECT_NAME} robot_manager_base robot_manager_node_extended robot_manager_node_rsi_only
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)

  # Test-only plugin with the mxA status fake. It lives in a private ament prefix under the
  # build directory, so it is never installed and no shipped description can select it.
  set(TEST_PLUGINS kuka_rsi_driver_test_plugins)
  set(TEST_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/test_prefix)
  add_library(${TEST_PLUGINS} SHARED
    test_plugin/mxa_status_fake.cpp
    test_plugin/hardware_interface_mxa_status_fake.cpp
  )
  target_include_directories(${TEST_PLUGINS} PRIVATE test_plugin)
  target_link_libraries(${TEST_PLUGINS} ${PROJECT_NAME})
  set_target_properties(${TEST_PLUGINS} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${TEST_PREFIX}/lib)
  file(WRITE ${TEST_PREFIX}/share/ament_index/resource_index/packages/${TEST_PLUGINS} "")
  file(WRITE
    ${TEST_PREFIX}/share/ament_index/resource_index/hardware_interface__pluginlib__plugin/${TEST_PLUGINS}
    "share/${TEST_PLUGINS}/mxa_status_fake_hardware_interface.xml")
  configure_file(test_plugin/mxa_status_fake_hardware_interface.xml
    ${TEST_PREFIX}/share/${TEST_PLUGINS}/mxa_status_fake_hardware_interface.xml COPYONLY)
  file(WRITE ${TEST_PREFIX}/share/${TEST_PLUGINS}/package.xml
    "<package format=\"3\"><name>${TEST_PLUGINS}</name></package>\n")
  set(TEST_PLUGIN_ENV APPEND_ENV AMENT_PREFIX_PATH=${TEST_PREFIX})

  add_launch_test(test/test_driver_startup_rsi_only.py)
  add_launch_test(test/test_driver_startup_eki_rsi.py)
  add_launch_test(test/test_driver_startup_mxa_rsi.py)
  add_launch_test(test/test_driver_startup_with_external_axis.py)
  add_launch_test(test/test_driver_activation.py)
  add_launch_test(test/test_driver_activation_dual_arm.py)
  add_launch_test(test/test_streams_without_event_broadcaster.py)
  add_launch_test(test/test_loop_timing.py)
  add_launch_test(test/test_configure_times_out_without_mxa.py)
  add_launch_test(test/test_mxa_status_fake.py ${TEST_PLUGIN_ENV})
  add_launch_test(test/test_mxa_status_fake_active.py ${TEST_PLUGIN_ENV})
  add_launch_test(test/test_activates_without_kroshu_controllers.py ${TEST_PLUGIN_ENV})
  add_launch_test(test/test_on_activate_sees_fresh_status.py ${TEST_PLUGIN_ENV})
  add_launch_test(test/test_exports_no_runtime_config.py
    TARGET test_exports_no_runtime_config_mxa_rsi ARGS "driver_version:=mxa_rsi")
  add_launch_test(test/test_exports_no_runtime_config.py
    TARGET test_exports_no_runtime_config_eki_rsi ARGS "driver_version:=eki_rsi")
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

install(DIRECTORY config launch test
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/generate_krc_rsi_config.py
  scripts/rsi_monitor_node.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
