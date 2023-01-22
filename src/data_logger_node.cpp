#include "data_logger/data_logger_node.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

using namespace message_filters;
using namespace std::placeholders;

dl::DataLoggerNode::DataLoggerNode(const rclcpp::NodeOptions& options) : Node("data_logger", options) {
    // Filter wraps our subscriptions
    auto image_sub = Subscriber<sensor_msgs::msg::Image>(this,
                                                         "/camera/mid/rgb");  //TODO make sure this is the correct QoS
    auto ack_sub = Subscriber<ackermann_msgs::msg::AckermannDrive>(this, "/odom_ack");

    // Approx sync the topics
    Synchronizer<sync_policy> sync{sync_policy(10), image_sub, ack_sub};
    sync.registerCallback(std::bind(&dl::DataLoggerNode::handle_training_data, this, _1, _2));
}

void dl::DataLoggerNode::handle_training_data(sensor_msgs::msg::Image::ConstSharedPtr image,
                                              ackermann_msgs::msg::AckermannDrive::ConstSharedPtr state) {
    //TODO write to CSV and save image
}
