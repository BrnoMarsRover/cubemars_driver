#include <rclcpp/rclcpp.hpp>

#include <cubemars_driver/CubeMarsNode.h>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CubeMarsNode>()->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
