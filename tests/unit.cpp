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

TEST(DataLoggerTests, InvalidDataIsNotAllowed) {
    rclcpp::NodeOptions opts;
    dl::DataLoggerNode node{opts};

    // Setup with non-zero but valid values
    ackermann_msgs::msg::AckermannDrive valid_state{};
    valid_state.acceleration = 2;
    valid_state.speed = 1;
    valid_state.steering_angle = 1.2;

    auto invalid_steering = std::make_unique<ackermann_msgs::msg::AckermannDrive>(valid_state);
    invalid_steering->steering_angle = -30;
    EXPECT_FALSE(node.validate_data(invalid_steering));
    invalid_steering->steering_angle = 30;
    EXPECT_FALSE(node.validate_data(invalid_steering));
    // Weird FP values
    invalid_steering->steering_angle = NAN;
    EXPECT_FALSE(node.validate_data(invalid_steering));
    invalid_steering->steering_angle = INFINITY;
    EXPECT_FALSE(node.validate_data(invalid_steering));
    invalid_steering->steering_angle = -INFINITY;
    EXPECT_FALSE(node.validate_data(invalid_steering));

    auto invalid_throttle = std::make_unique<ackermann_msgs::msg::AckermannDrive>(valid_state);
    invalid_throttle->acceleration = 20;
    EXPECT_FALSE(node.validate_data(invalid_throttle));
    invalid_throttle->acceleration = -20;
    EXPECT_FALSE(node.validate_data(invalid_throttle));
    // Weird FP values
    invalid_throttle->acceleration = NAN;
    EXPECT_FALSE(node.validate_data(invalid_throttle));
    invalid_throttle->acceleration = INFINITY;
    EXPECT_FALSE(node.validate_data(invalid_throttle));
    invalid_throttle->acceleration = -INFINITY;
    EXPECT_FALSE(node.validate_data(invalid_throttle));

    auto invalid_encoder = std::make_unique<ackermann_msgs::msg::AckermannDrive>(valid_state);
    invalid_encoder->speed = -0.9;
    EXPECT_FALSE(node.validate_data(invalid_encoder));
    // Weird FP values
    invalid_encoder->speed = NAN;
    EXPECT_FALSE(node.validate_data(invalid_encoder));
    invalid_encoder->speed = INFINITY;
    EXPECT_FALSE(node.validate_data(invalid_encoder));
    invalid_encoder->speed = -INFINITY;
    EXPECT_FALSE(node.validate_data(invalid_encoder));
}

TEST(DataLoggerTests, ValidDataIsAllowed) {
    rclcpp::NodeOptions opts;
    dl::DataLoggerNode node{opts};

    ackermann_msgs::msg::AckermannDrive valid_state{};

    auto valid_steering = std::make_unique<ackermann_msgs::msg::AckermannDrive>(valid_state);
    valid_steering->steering_angle = 2.0;
    EXPECT_TRUE(node.validate_data(valid_steering));
    valid_steering->steering_angle = 1.2234234 / 9;
    EXPECT_TRUE(node.validate_data(valid_steering));
    valid_steering->steering_angle = 0;
    EXPECT_TRUE(node.validate_data(valid_steering));
    valid_steering->steering_angle = -1.2234234 / 9;
    EXPECT_TRUE(node.validate_data(valid_steering));
    valid_steering->steering_angle = -2.0;
    EXPECT_TRUE(node.validate_data(valid_steering));

    auto valid_throttle = std::make_unique<ackermann_msgs::msg::AckermannDrive>(valid_state);
    valid_throttle->acceleration = 0.0;
    EXPECT_TRUE(node.validate_data(valid_throttle));
    valid_throttle->acceleration = -5.234234 / 9;
    EXPECT_TRUE(node.validate_data(valid_throttle));
    valid_throttle->acceleration = -10.0;
    EXPECT_TRUE(node.validate_data(valid_throttle));
    valid_throttle->acceleration = 5.234234 / 9;
    EXPECT_TRUE(node.validate_data(valid_throttle));
    valid_throttle->acceleration = 10.0;
    EXPECT_TRUE(node.validate_data(valid_throttle));

    auto valid_encoder = std::make_unique<ackermann_msgs::msg::AckermannDrive>(valid_state);
    valid_encoder->speed = 0.0;
    EXPECT_TRUE(node.validate_data(valid_encoder));
    valid_encoder->speed = 10.0;
    EXPECT_TRUE(node.validate_data(valid_encoder));
}

int main(int argc, char** argv) {
    rclcpp::init(0, nullptr);

    ::testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();

    rclcpp::shutdown();
    return res;
}