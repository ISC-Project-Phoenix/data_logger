#pragma once

#include <filesystem>
#include <fstream>

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

namespace dl {

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

    /// Max steering wheel angle, in radians.
    float max_steering_rad;

    /// Speed that represents the throttle being pressed 100%.
    float max_throttle_speed;

    /// Speed that represents the brake being pressed 100%.
    float max_brake_speed;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ack_sub;

    // Queues we use to 'sync' messages
    std::list<sensor_msgs::msg::Image::SharedPtr> image_queue;
    std::list<ackermann_msgs::msg::AckermannDrive::SharedPtr> ack_queue;

public:
    /// CSV Headers
    static constexpr const char* HEADERS =
        "image_file_name, steering_angle, throttle, brake, linux_time, velocity, velocity_x, velocity_y, "
        "velocity_z, position_x, position_y, position_z \n";

    DataLoggerNode(const rclcpp::NodeOptions& options);

    /// Writes training data to disk. This image and state should be as synced as possible.
    /// Note: this is currently measured to take about 25ms, so this is safe to call in a callback.
    void handle_training_data(sensor_msgs::msg::Image::SharedPtr image,
                              ackermann_msgs::msg::AckermannDrive::SharedPtr state);

    /// Camera callback
    void camera_cb(sensor_msgs::msg::Image::SharedPtr image);

    /// Ackermann odom callback
    void ack_cb(ackermann_msgs::msg::AckermannDrive::SharedPtr state);

    /// Returns the absolute path to the passed string.
    static std::filesystem::path normalise_path(std::string_view s);

    /// Prepares the filesystem for logging, including creating directories and the csv.
    void setup_data_folder();

    /// Formats data into a csv line.
    std::string create_csv_line(const std::string_view& image_filename, std::time_t stamp,
                                const ackermann_msgs::msg::AckermannDrive::ConstSharedPtr& state) const;

    /// Returns the path to the data folder containing the run data folders.
    const std::filesystem::path& getOuterDataFolder() const;

    /// Returns the path to this runs data folder.
    const std::filesystem::path& getDataFolder() const;
};
}  // namespace dl