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
    std::filesystem::path data_folder;

    /// Path to the folder we are actually writing images to.
    std::filesystem::path run_folder;

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
    [[maybe_unused]] static constexpr const char* HEADERS =
        "image_file_name, steering_angle, throttle, brake, linux_time, velocity, velocity_x, "
        "velocity_y, "
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

    /// Formats data into a csv line. Data is assumed to be valid.
    std::string create_csv_line(const std::string_view& image_filename, std::time_t stamp,
                                const ackermann_msgs::msg::AckermannDrive::ConstSharedPtr& state) const;

    /// Checks if data is in a valid state for writing to the CSV. Returns true if valid.
    /// This function is generic across pointers to AckermannDrive.
    template <class T>
    bool validate_data(const T& state) const;

    /// Returns the path to the data folder containing the run data folders.
    const std::filesystem::path& getDataFolder() const;

    /// Returns the path to this runs data folder.
    const std::filesystem::path& getRunFolder() const;
};
}  // namespace dl

template <class T>
bool dl::DataLoggerNode::validate_data(const T& state) const {
    if (abs(state->steering_angle) > abs(this->max_steering_rad)) {
        RCLCPP_WARN(this->get_logger(), "Steering angle %f is greater than the max steering angle!",
                    state->steering_angle);
        return false;
    } else if (state->acceleration > this->max_throttle_speed) {
        RCLCPP_WARN(this->get_logger(), "Throttle %f is greater than the max throttle value!", state->acceleration);
        return false;
    } else if (state->acceleration < this->max_brake_speed) {
        RCLCPP_WARN(this->get_logger(), "Brake %f is less than the max brake value!", state->acceleration);
        return false;
    } else if (state->speed < 0) {
        RCLCPP_WARN(this->get_logger(), "Encoder speed %f is less than 0!", state->speed);
        return false;
    } else if (state->speed == INFINITY) {
        RCLCPP_WARN(this->get_logger(), "Encoder speed is inf! Phoenix cannot go that fast yet :(");
        return false;
    } else if (std::isnan(state->steering_angle) || std::isnan(state->acceleration) || std::isnan(state->speed)) {
        RCLCPP_WARN(this->get_logger(), "Ackermann odom value was nan!");
        return false;
    }

    return true;
}