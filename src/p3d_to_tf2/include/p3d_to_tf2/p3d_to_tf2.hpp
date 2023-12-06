#pragma once

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class FramePublisher : public rclcpp::Node
{
    public:
        FramePublisher();

    private:
        void handle_p3d_pos(const std::shared_ptr<nav_msgs::msg::Odometry> msg);
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::string turtlename_;

};




