#pragma once

#include "rclcpp/rclcpp.hpp"

namespace dl {
    class DataLoggerNode : public rclcpp::Node {
    public:
        DataLoggerNode(const rclcpp::NodeOptions &options);
    };
}