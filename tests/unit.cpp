#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "data_logger/data_logger_node.hpp"

TEST(DataLoggerTests, DataFolderParamWorks) {
    rclcpp::NodeOptions opts;

    // Change the path param
    auto path = "./not_normal_path";
    opts.append_parameter_override("data_path", path);

    dl::DataLoggerNode node{opts};

    auto should_be = (std::filesystem::current_path() / path).lexically_normal();

    // This can fail if our normalisation does not remove the './'
    EXPECT_EQ(node.getDataFolder(), should_be);
    // Run folder should be in data folder
    EXPECT_EQ(node.getRunFolder().parent_path(), should_be);
}

int main(int argc, char** argv) {
    rclcpp::init(0, nullptr);

    ::testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();

    rclcpp::shutdown();
    return res;
}