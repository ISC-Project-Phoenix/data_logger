#include <cstdio>

#include "data_logger/data_logger_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto node = std::make_shared<dl::DataLoggerNode>(options);
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
