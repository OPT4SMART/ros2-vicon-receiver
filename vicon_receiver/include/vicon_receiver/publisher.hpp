#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "vicon_msgs/msg/position.hpp"

// Struct used to hold segment data to transmit to the Publisher class.
struct PositionStruct
{
    double translation[3];
    double rotation[4];
    std::string subject_name;
    std::string segment_name;
    std::string translation_type;
    unsigned int frame_number;

} typedef PositionStruct;


// Class that allows segment data to be published in a ROS2 topic.
class Publisher
{
private:
    rclcpp::Publisher<vicon_msgs::msg::Position>::SharedPtr position_publisher_;

public:
    bool is_ready = false;

    Publisher(std::string topic_name, rclcpp::Node* node);

    // Publishes the given position in the ROS2 topic whose name is indicated in
    // the constructor.
    void publish(PositionStruct p);
};

#endif