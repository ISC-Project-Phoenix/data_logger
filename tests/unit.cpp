#include <gtest/gtest.h>

#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

#include "data_logger/data_logger_node.hpp"

TEST(Some, test) {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions opts;
    dl::DataLoggerNode node{opts};
    rclcpp::shutdown();
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}