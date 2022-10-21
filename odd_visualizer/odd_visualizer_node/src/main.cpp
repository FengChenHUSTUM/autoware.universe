#include <rclcpp/rclcpp.hpp>
#include "odd_visualizer_node.hpp"

#include <memory>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    auto node = std::make_shared<odd_visualizer::OddVisualizer>("odd_visualizer_node", node_options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}