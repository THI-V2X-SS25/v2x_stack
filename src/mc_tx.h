#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <etsi_its_mcm_thi_prima_msgs/msg/mcm.hpp>
#include <etsi_its_mcm_thi_prima_coding/asn_MCM.h>
#include <cstdint>

namespace v2x_stack_btp
{
class CaRxNode : public rclcpp::Node
{
public:
    explicit CaRxNode(const rclcpp::NodeOptions & options);
    void onPosition(const sensor_msgs::msg::NavSatFix::ConstSharedPtr);
    void onHeading(const std_msgs::msg::Float64::ConstSharedPtr);
    void onVelocity(const std_msgs::msg::Float64::ConstSharedPtr);

    void publish(); // private or public? 

private:

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_navsat_fix_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_heading_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_velocity_;

    sensor_msgs::msg::NavSatFix::ConstSharedPtr position_;
    std_msgs::msg::Float64::ConstSharedPtr heading_;
    std_msgs::msg::Float64::ConstSharedPtr velocity_;

    std::shared_ptr<rclcpp::Publisher<etsi_its_mcm_thi_prima_msgs::msg::MCM>> pub_mcm_;
    rclcpp::Node::SharedPtr node_;
};

}