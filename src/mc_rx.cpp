// #include "mc_message.h"
#include "mc_rx.h"
#include <vanetza/btp/ports.hpp>

namespace v2x_stack_btp
{

CaRxNode::CaRxNode(const rclcpp::NodeOptions & options)
: Node("mc_rx_node", options)
{
}

void CaRxNode::onIndication(msg::BtpDataIndication::ConstSharedPtr indication)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Indication MC");
    if (indication->btp_type == msg::BtpDataIndication::BTP_TYPE_B && indication->destination_port == 2020)  // port tbd.
    {
        //vanetza::asn1::r1::Mcm mcm;
        etsi_its_mcm_thi_prima_coding::asn_MCM mcm;
        const std::vector<unsigned char>& payload = indication->data;
        const uint8_t* buffer = payload.data();

        auto ok = vanetza::asn1::decode_per((asn_TYPE_descriptor_t&)(asn_DEF_asn_MCM), (void**)(&mcm), (const void*)(buffer), indication->data.size());
    
        if (ok)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Well decoded MC");
            publish(mcm);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ASN.1 decoding failed, drop received MCM");
        }
    }
}

//void CaRxNode::publish(const vanetza::asn1::r1::Mcm asn1)
void CaRxNode::publish(const etsi_its_mcm_thi_prima_coding::asn_MCM asn1)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Entering publish method");
    std::string error_msg;
    auto msg = convertMcm(asn1, &error_msg);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Try to Publish MCM");

    if (msg)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "There is MCM");
        node_ = std::make_shared<rclcpp::Node>("mcm_rx");
        pub_mcm_ = node_->create_publisher<etsi_its_mcm_thi_prima_msgs::msg::MCM>("mcm_received", 20);
    
        pub_mcm_->publish(*msg);
    } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MCM not correct");
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Error cause: %s", error_msg.c_str());
    }
}

} // namespace v2x_stack_btp

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting MC RX");

    auto node = std::make_shared<v2x_stack_btp::CaRxNode>(rclcpp::NodeOptions());
    auto subscription = node->create_subscription<v2x_stack_btp::msg::BtpDataIndication>("btp_data", 20, std::bind(&v2x_stack_btp::CaRxNode::onIndication, node, std::placeholders::_1));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
