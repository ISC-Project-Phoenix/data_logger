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

    /// Handle to data csv
    std::ofstream csv;

public:
    /// CSV Headers
    static constexpr const char* HEADERS =
        "image_file_name, steering_angle, throttle, brake, linux_time, velocity, velocity_x, velocity_y, "
        "velocity_z, position_x, position_y, position_z";

    DataLoggerNode(const rclcpp::NodeOptions& options);

    /// Handles collected training data. This image and state should be as synced as possible.
    void handle_training_data(sensor_msgs::msg::Image::ConstSharedPtr image,
                              ackermann_msgs::msg::AckermannDrive::ConstSharedPtr state);

    /// Returns the absolute path to the passed string.
    static std::filesystem::path normalise_path(std::string_view s);

    /// Prepares the filesystem for logging, including creating directories and the csv.
    void setup_data_folder();

    /// Returns the path to the data folder containing the run data folders.
    const std::filesystem::path& getOuterDataFolder() const;

    /// Returns the path to this runs data folder.
    const std::filesystem::path& getDataFolder() const;
};
}  // namespace dl