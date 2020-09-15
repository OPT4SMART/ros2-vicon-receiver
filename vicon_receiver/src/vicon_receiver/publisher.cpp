#include "vicon_receiver/publisher.hpp"

Publisher::Publisher(std::string topic_name, rclcpp::Node* node)
{
    position_publisher_ = node->create_publisher<vicon_receiver::msg::Position>(topic_name, 10);
    is_ready = true;
}

void Publisher::publish(PositionStruct p)
{
    auto msg = std::make_shared<vicon_receiver::msg::Position>();
    msg->x_trans = p.translation[0];
    msg->y_trans = p.translation[1];
    msg->z_trans = p.translation[2];
    msg->x_rot = p.rotation[0];
    msg->y_rot = p.rotation[1];
    msg->z_rot = p.rotation[2];
    msg->w = p.rotation[3];
    msg->subject_name = p.subject_name;
    msg->segment_name = p.segment_name;
    msg->frame_number = p.frame_number;
    msg->translation_type = p.translation_type;
    position_publisher_->publish(*msg);
}
