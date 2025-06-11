#include "mc_tx.h"
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
    if (!position_ || !heading_ || !velocity_) {
        RCLCPP_WARN(this->get_logger(), "Not all data available yet. Skipping publish.");
        return;
    }

    auto msg = std::make_shared<etsi_its_mcm_thi_prima_msgs::msg::MCM>();

    // add timestamp 
    auto& basic_container = msg->mcm.mcm_parameters.basic_container_mcm;
    basic_container.station_type.value = 5;
    basic_container.reference_position.latitude.value = position_->latitude;
    basic_container.reference_position.longitude.value = position_->longitude;
    basic_container.reference_position.altitude.altitude_value.value = position_->altitude;
    basic_container.reference_position.altitude.altitude_confidence.value = 15;

    auto& intention_sharing_container = msg->mcm.mcm_parameters.intention_sharing_container;
    intention_sharing_container.heading.heading_value.value = heading_->data;
    intention_sharing_container.heading.heading_confidence.value = 127;
    intention_sharing_container.speed.speed_value.value = velocity_->data;
    intention_sharing_container.speed.speed_confidence.value = 127;
    intention_sharing_container.drive_direction.value = 0;

    auto veh_traj = etsi_its_mcm_thi_prima_msgs::msg::TrajectoryPointMCM();
    veh_traj.delta_longitudinal_position.value = 0;
    veh_traj.delta_lateral_position.value = 0;
    veh_traj.delta_heading.value = 0;
    veh_traj.delta_time.value = 1;

    intention_sharing_container.planned_trajectory.array.push_back(veh_traj);

    // publish the message
    RCLCPP_INFO(this->get_logger(), "Publishing MCM message");
    if (!pub_mcm_) {
        pub_mcm_ = this->create_publisher<etsi_its_mcm_thi_prima_msgs::msg::MCM>("/etsi_its_conversion/mcm_thi_prima/in", 20);
    }
    pub_mcm_->publish(*msg);
}

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting MC TX node");

    auto node = std::make_shared<v2x_stack_btp::CaRxNode>(rclcpp::NodeOptions());
    auto sub_navsat_fix = node->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/genesys/adma/fix", 20, std::bind(&v2x_stack_btp::CaRxNode::onPosition, node, std::placeholders::_1));
    auto sub_heading = node->create_subscription<std_msgs::msg::Float64>(
        "/genesys/adma/heading", 20, std::bind(&v2x_stack_btp::CaRxNode::onHeading, node, std::placeholders::_1));
    auto sub_velocity = node->create_subscription<std_msgs::msg::Float64>(
        "/genesys/adma/velocity", 20, std::bind(&v2x_stack_btp::CaRxNode::onVelocity, node, std::placeholders::_1));
    
    // timer to publish MCM
    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&v2x_stack_btp::CaRxNode::publish, node));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}
