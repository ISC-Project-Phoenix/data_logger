#include "data_logger/data_logger_node.hpp"

#include <chrono>

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "rclcpp/qos.hpp"

using namespace message_filters;
using namespace std::placeholders;
using namespace std::literals::string_literals;

dl::DataLoggerNode::DataLoggerNode(const rclcpp::NodeOptions& options) : Node("data_logger", options) {
    // Filter wraps our subscriptions
    auto image_sub = Subscriber<sensor_msgs::msg::Image>(this, "/camera/mid/rgb");
    auto ack_sub = Subscriber<ackermann_msgs::msg::AckermannDrive>(this, "/odom_ack");

    // Approx sync the topics
    Synchronizer<sync_policy> sync{sync_policy(10), image_sub, ack_sub};
    sync.registerCallback(
        std::bind(&dl::DataLoggerNode::handle_training_data, this, _1, _2));  //TODO do we need to save this in a field?

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

void dl::DataLoggerNode::handle_training_data(sensor_msgs::msg::Image::ConstSharedPtr image,
                                              ackermann_msgs::msg::AckermannDrive::ConstSharedPtr state) {
    //TODO write to CSV and save image
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

// For testing
const std::filesystem::path& dl::DataLoggerNode::getOuterDataFolder() const { return outer_data_folder; }
const std::filesystem::path& dl::DataLoggerNode::getDataFolder() const { return data_folder; }
