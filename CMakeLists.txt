cmake_minimum_required(VERSION 3.5)
project(cubemars_driver)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++20
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

include_directories(include)

# | ---------- add library ----------
# Pure SocketCAN protocol layer -- no ROS dependency, so it can be reused
# from a plain C++ program or a test harness.
add_library(cubemars src/CubeMars.cpp)
target_include_directories(cubemars PUBLIC include)
# linked into the ros2_control plugin (a shared library) as well
set_target_properties(cubemars PROPERTIES POSITION_INDEPENDENT_CODE ON)

# | ---------- add executable ----------
add_executable(cubemars_node src/cubemars_node.cpp src/CubeMarsNode.cpp)
target_include_directories(cubemars_node PUBLIC include)
ament_target_dependencies(
        cubemars_node
        rclcpp
        sensor_msgs
)

# Link the library to the executable
target_link_libraries(cubemars_node cubemars)

install(TARGETS cubemars_node cubemars
        DESTINATION lib/${PROJECT_NAME})

# | ---------- ros2_control: cubemars_driver/CubeMarsSystem ----------
add_library(cubemars_ros2_control SHARED src/CubeMarsSystem.cpp)
target_link_libraries(cubemars_ros2_control cubemars)
ament_target_dependencies(cubemars_ros2_control hardware_interface pluginlib rclcpp rclcpp_lifecycle)
pluginlib_export_plugin_description_file(hardware_interface cubemars_driver_plugins.xml)
install(TARGETS cubemars_ros2_control
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

# Install
install(
        DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME})

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    set(ament_cmake_copyright_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_export_include_directories(
        include
)
ament_export_libraries(
        cubemars
)

ament_package()
