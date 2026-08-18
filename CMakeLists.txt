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

include_directories(include)

# | ---------- add library ----------
# Pure SocketCAN protocol layer -- no ROS dependency, so it can be reused
# from a plain C++ program or a test harness.
add_library(cubemars src/CubeMars.cpp)
target_include_directories(cubemars PUBLIC include)

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
