#include "mcm_tx.h"
#include <vanetza/btp/ports.hpp>

namespace v2x_stack_btp
{

CaRxNode::CaRxNode(const rclcpp::NodeOptions & options)
: Node("mc_tx_node", options){
}

void CaRxNode::onPosition(sensor_msgs::msg::NavSatFix::ConstSharedPtr position)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Position MC");
    if (position->status.status != sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Well decoded position");
        position_ = position;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No fix position, drop received MCM");
    }

}

void CaRxNode::onHeading(std_msgs::msg::Float64::ConstSharedPtr heading)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Heading MC");
    heading_ = heading;
}

void CaRxNode::onVelocity(std_msgs::msg::Float64::ConstSharedPtr velocity)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Velocity MC");
    velocity_ = velocity;
}

void CaRxNode::publish()
{
    auto msg = std::make_shared<etsi_its_mcm_thi_prima_msgs::msg::MCM>();
    // add timestamp 
    //msg->mcm.generation_delta_time = deltaTime;
    auto& basic_container = msg->mcm.mcm_parameters.basic_container_mcm;
    basic_container.station_type.value = 5; // 5 = Passenger car
    basic_container.reference_position.latitude.value = position_->latitude;
    basic_container.reference_position.longitude.value = position_->longitude;
    basic_container.reference_position.altitude.altitude_value.value = position_->altitude;
    basic_container.reference_position.altitude.altitude_confidence.value = 15; // unavailable  !!!!pls change!!!

    auto& intention_sharing_container = msg->mcm.mcm_parameters.intention_sharing_container;
    intention_sharing_container.heading.heading_value.value = heading_->data;
    intention_sharing_container.heading.heading_confidence.value = 127; // unavailable  !!!!pls change!!!
    intention_sharing_container.speed.speed_value.value = velocity_->data;
    intention_sharing_container.speed.speed_confidence.value = 127; // unavailable  !!!!pls change!!!
    intention_sharing_container.drive_direction.value = 0; // forward


    // publish the message
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Try to Publish tx_MCM");
    node_ = std::make_shared<rclcpp::Node>("mcm_tx");
    pub_mcm_ = node_->create_publisher<etsi_its_mcm_thi_prima_msgs::msg::MCM>("mcm_transmitted", 20);
    pub_mcm_->publish(*msg);
    
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting MC TX node");

    auto node = std::make_shared<CaRxNode>(rclcpp::NodeOptions());
    auto sub_navsat_fix = node->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/adma/fix", 20, std::bind(&v2x_stack_btp::&CaRxNode::onPosition, node, std::placeholders::_1));
    auto sub_heading = node->create_subscription<std_msgs::msg::Float64>(
        "/adma/heading", 20, std::bind(&v2x_stack_btp::&CaRxNode::onHeading, node, std::placeholders::_1));
    auto sub_velocity = node->create_subscription<std_msgs::msg::Float64>(
        "/adma/velocity", 20, std::bind(&v2x_stack_btp::&CaRxNode::onVelocity, node, std::placeholders::_1));
    
    // timer to publish MCM
    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&v2x_stack_btp::CaRxNode::publish, node));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}
}
