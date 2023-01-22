#pragma once

#include <filesystem>
#include <fstream>

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "message_filters/sync_policies/approximate_time.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

namespace dl {
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, ackermann_msgs::msg::AckermannDrive>
    sync_policy;

class DataLoggerNode : public rclcpp::Node {
private:
    /// Path to the containing data folder, not the folder we are writing images to.
    std::filesystem::path outer_data_folder;
    /// Path to the folder we are actually writing images to.
    std::filesystem::path data_folder;

    /// Publishes current data_folder
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr path_pub;

public:
    DataLoggerNode(const rclcpp::NodeOptions& options);

    /// Handles collected training data. This image and state should be as synced as possible.
    void handle_training_data(sensor_msgs::msg::Image::ConstSharedPtr image,
                              ackermann_msgs::msg::AckermannDrive::ConstSharedPtr state);
};
}  // namespace dl