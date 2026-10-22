cmake_minimum_required(VERSION 3.8)
project(cubemars_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies; every one is also in package.xml
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# | ---------- CubeMars servo protocol over SocketCAN, plain C++ without ROS ---------- |
add_library(cubemars SHARED src/CubeMars.cpp)
target_include_directories(cubemars PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# | ---------- ros2_control: cubemars_driver/CubeMarsSystem ---------- |
add_library(cubemars_ros2_control SHARED src/CubeMarsSystem.cpp)
target_link_libraries(cubemars_ros2_control PUBLIC
  cubemars
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
pluginlib_export_plugin_description_file(hardware_interface cubemars_driver_plugins.xml)

# | ---------- install ---------- |
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(TARGETS cubemars cubemars_ros2_control
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# zero calibration from the live bus, see the README
install(PROGRAMS tools/calibrate_zero.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

ament_package()
