#include "../include/p3d_to_tf2/p3d_to_tf2.hpp"
FramePublisher::FramePublisher() : Node("p3d_tf2_frame_publisher")
{
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    std::string topic_name = "/p3d/odom_p3d";

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      topic_name, 10,
      std::bind(&FramePublisher::handle_p3d_pos, this, std::placeholders::_1));
}


void FramePublisher::handle_p3d_pos(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
{
geometry_msgs::msg::TransformStamped t;

// Read message content and assign it to
// corresponding tf variables
t.header.stamp = this->get_clock()->now();
t.header.frame_id = "world";
t.child_frame_id = "base_link";

// Turtle only exists in 2D, thus we get x and y translation
// coordinates from the message and set the z coordinate to 0
t.transform.translation.x = msg->pose.pose.position.x;
t.transform.translation.y = msg->pose.pose.position.y;
t.transform.translation.z = msg->pose.pose.position.z;

// For the same reason, turtle can only rotate around one axis
// and this why we set rotation in x and y to 0 and obtain
// rotation in z axis from the message
t.transform.rotation.x = msg->pose.pose.orientation.x;
t.transform.rotation.y = msg->pose.pose.orientation.y;
t.transform.rotation.z = msg->pose.pose.orientation.z;
t.transform.rotation.w = msg->pose.pose.orientation.w;

// Send the transformation
tf_broadcaster_->sendTransform(t);
}