#pragma once

#include <rclcpp/rclcpp.hpp>
//#include <vanetza/asn1/mcm.hpp>  // currently missing
#include <v2x_stack_btp/msg/btp_data_indication.hpp>
#include <etsi_its_mcm_thi_prima_msgs/msg/mcm.hpp>
#include <etsi_its_mcm_thi_prima_coding/asn_MCM.h>
#include <cstdint>

namespace v2x_stack_btp
{

class CaRxNode : public rclcpp::Node
{
public:
    explicit CaRxNode(const rclcpp::NodeOptions & options);
    void onIndication(const msg::BtpDataIndication::ConstSharedPtr);

private:
    
    //void publish(const vanetza::asn1::r1::Mcm);
    void publish(const etsi_its_mcm_thi_prima_coding::asn_MCM);

    uint16_t port_;
    rclcpp::Subscription<msg::BtpDataIndication>::SharedPtr sub_btp_;
    std::shared_ptr<rclcpp::Publisher<etsi_its_mcm_thi_prima_msgs::msg::MCM>> pub_mcm_;
    rclcpp::Node::SharedPtr node_;
};

} // namespace v2x_stack_btp