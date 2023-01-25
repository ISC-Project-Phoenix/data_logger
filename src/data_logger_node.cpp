#include "data_logger/data_logger_node.hpp"

#include <chrono>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "rclcpp/qos.hpp"

using namespace std::placeholders;
using namespace std::literals::string_literals;

dl::DataLoggerNode::DataLoggerNode(const rclcpp::NodeOptions& options) : Node("data_logger", options) {
    // Random configs
    this->max_brake_speed = this->declare_parameter<float>("max_brake_speed", -10.0);
    this->max_throttle_speed = this->declare_parameter<float>("max_throttle_speed", 10.0);
    this->max_steering_rad = this->declare_parameter<float>("max_steering_rad", 2.0);

    image_sub = this->create_subscription<sensor_msgs::msg::Image>("/camera/mid/rgb", rclcpp::QoS(10).reliable(),
                                                                   std::bind(&DataLoggerNode::camera_cb, this, _1));

    ack_sub = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
        "/odom_ack", rclcpp::SensorDataQoS{}, std::bind(&DataLoggerNode::ack_cb, this, _1));

    // Get path to outer data folder
    try {
        auto raw_data_path = this->declare_parameter<std::string>("data_path", "./training_data"s);
        this->outer_data_folder = dl::DataLoggerNode::normalise_path(raw_data_path);
    } catch (...) {
        RCLCPP_FATAL(this->get_logger(), "Failed to parse data_path into absolute path!");
        std::exit(1);
    }
    RCLCPP_INFO(this->get_logger(), "Using data folder path: %s", outer_data_folder.string().c_str());

    // Create dir and csv
    this->setup_data_folder();
    RCLCPP_INFO(this->get_logger(), "Using run data folder path: %s", data_folder.string().c_str());

    // Publish path to the inner folder we just made, reliably
    {
        this->path_pub = this->create_publisher<std_msgs::msg::String>(
            "/run_folder", rclcpp::QoS(1).keep_last(1).reliable().transient_local());
        std_msgs::msg::String msg{};
        msg.data = this->data_folder.string();
        path_pub->publish(msg);
    }
}

void dl::DataLoggerNode::camera_cb(const sensor_msgs::msg::Image::SharedPtr image) {
    // Bound queue size since images can get very big
    if (image_queue.size() > 10) {
        // Clearing makes more sense for us since we don't want to hold old images, just buffer images so that IO doesn't cause too much data loss
        this->image_queue.clear();
    }

    this->image_queue.push_back(image);

    // Write to disk when both queues have a message to write
    if (!this->image_queue.empty() && !this->ack_queue.empty()) {
        this->handle_training_data(image_queue.front(), ack_queue.front());
        image_queue.pop_front();
        ack_queue.pop_front();
    }
}

void dl::DataLoggerNode::ack_cb(const ackermann_msgs::msg::AckermannDrive::SharedPtr state) {
    this->ack_queue.push_back(state);

    // Write to disk when both queues have a message to write
    if (!this->image_queue.empty() && !this->ack_queue.empty()) {
        this->handle_training_data(image_queue.front(), ack_queue.front());
        image_queue.pop_front();
        ack_queue.pop_front();
    }
}

void dl::DataLoggerNode::handle_training_data(const sensor_msgs::msg::Image::SharedPtr image,
                                              const ackermann_msgs::msg::AckermannDrive::SharedPtr state) {
    // Neither message has a stamp, so stamp here
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream time_str;
    time_str << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%H-%M-%S");
    auto image_filename = time_str.str() + std::to_string(ms.time_since_epoch().count()) + ".jpg";

    // Write image to disk
    auto cv_img = cv_bridge::toCvShare(image);
    cv::imwrite(this->data_folder / image_filename, cv_img->image);

    // Write to csv
    auto csv_line = this->create_csv_line(image_filename, in_time_t, state);
    csv << csv_line;
}

std::filesystem::path dl::DataLoggerNode::normalise_path(std::string_view s) {
    std::filesystem::path path{s};
    auto abs = std::filesystem::weakly_canonical(s);
    return abs;
}

void dl::DataLoggerNode::setup_data_folder() {
    // Create outer folder
    std::filesystem::create_directories(this->outer_data_folder);

    // Create this runs folder according to time as per spec
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream folder;
    folder << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%H-%M-%S");

    this->data_folder = this->outer_data_folder / folder.str();

    // Create inner folder
    std::filesystem::create_directories(this->data_folder);

    // Create csv
    auto csv_path = this->data_folder / (folder.str() + ".csv");
    RCLCPP_INFO(this->get_logger(), "Using csv path: %s", csv_path.string().c_str());
    this->csv = std::ofstream(csv_path);
    // Write headers
    csv << dl::DataLoggerNode::HEADERS;
}

std::string dl::DataLoggerNode::create_csv_line(
    const std::string_view& image_filename, std::time_t stamp,
    const ackermann_msgs::msg::AckermannDrive::ConstSharedPtr& state) const {
    // Normalise values to 0-1 range
    auto steering_angle = state->steering_angle / this->max_steering_rad;
    float throttle = 0, brake = 0;

    // Handle throttle/brake ambiguity
    if (state->acceleration > 0) {
        throttle = state->acceleration / this->max_throttle_speed;
    } else if (state->acceleration < 0) {
        brake = state->acceleration / this->max_brake_speed;
    }

    auto velocity = state->speed;

    // Create row
    std::stringstream buf;

    buf << image_filename << ',' << steering_angle << ',' << throttle << ',' << brake << ',' << stamp << ','
        << velocity
        // This is vel_x
        << ',' << velocity;

    // Remaining fields are currently unused, these are v_y, v_z, and xyz positions.
    buf << "0,0,0,0,0 \n";

    return buf.str();
}

// For testing
const std::filesystem::path& dl::DataLoggerNode::getOuterDataFolder() const { return outer_data_folder; }

const std::filesystem::path& dl::DataLoggerNode::getDataFolder() const { return data_folder; }
