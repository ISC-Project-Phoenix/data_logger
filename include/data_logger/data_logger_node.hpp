#pragma once

#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "std_msgs/msg/string.hpp"

namespace dl {
    class DataLoggerNode : public rclcpp::Node {
    private:
        /// Path to the containing data folder, not the folder we are writing images to.
        std::filesystem::path outer_data_folder;
        /// Path to the folder we are actually writing images to.
        std::filesystem::path data_folder;

        /// Image stream
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
        /// Ackermann odom stream
        rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ack_sub;

        /// Publishes current data_folder
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr path_pub;

        //TODO add file handle for csv
    public:
        DataLoggerNode(const rclcpp::NodeOptions &options);
    };
}