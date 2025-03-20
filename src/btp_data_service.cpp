#include "btp_data_service.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "v2x_stack_btp/msg/cohda_ind.hpp"
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <vanetza/btp/header.hpp>
#include <vanetza/net/osi_layer.hpp>
#include <vanetza/geonet/serialization_buffer.hpp>

using namespace vanetza;

namespace v2x_stack_btp
{

BtpPublisher::BtpPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("btp_publisher_node", options)
{
    // Constructor logic
//    subscription_ = this->create_subscription<v2x_stack::msg::CohdaInd>(
//        "udp_data", 10, std::bind(&BtpPublisher::udpCallback, this, std::placeholders::_1));

    service_ = this->create_service<srv::BtpData>(
        "btp_indication", std::bind(&BtpPublisher::serviceCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void BtpPublisher::udpCallback(const v2x_stack_btp::msg::CohdaInd::SharedPtr msg)
{    
    //after: evaluate the DataIndication as vanetza::btp::DataIndication
    publish(msg);
    
}

void BtpPublisher::publish (const v2x_stack_btp::msg::CohdaInd::SharedPtr packet)
{    
    msg::BtpDataIndication btp_msg;
    //auto btp_msg = boost::make_shared<msg::BtpDataIndication>();

    
    //msg->header.stamp = node->get_clock()->now();

    //for destination port
    btp_msg.btp_type = packet->header.btp_type;
    btp_msg.destination_port_info = packet->header.dest_info;
    btp_msg.destination_port = packet->header.dest_port;
    btp_msg.its_aid = packet->header.aid;
    btp_msg.traffic_class.id = packet->header.traffic_class;
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "resize");
    int btpSize = packet->type.msg_length;
    btp_msg.data.resize(btpSize);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pass values");
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "done");
    
    std::copy(packet->payload.begin(), packet->payload.end(), btp_msg.data.begin());

    node_ = std::make_shared<rclcpp::Node>("btp_publisher");
    publisher_ = node_->create_publisher<msg::BtpDataIndication>("btp_data", 10);
    
    publisher_->publish(btp_msg);

}

bool BtpPublisher::serviceCallback(const std::shared_ptr<srv::BtpData::Request> request,
                                   std::shared_ptr<srv::BtpData::Response> response)
{
    RCLCPP_INFO(get_logger(), "Received BTP service request");

    return true;
}

} // namespace v2x_stack_btp

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting BTP Node");

    auto node = std::make_shared<v2x_stack_btp::BtpPublisher>(rclcpp::NodeOptions());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Create Node");

    auto subscription = node->create_subscription<v2x_stack_btp::msg::CohdaInd>("udp_data", 10, [node](const v2x_stack_btp::msg::CohdaInd::SharedPtr msg) {node->udpCallback(msg);});
    //auto subscription = node->create_subscription<v2x_stack::msg::CohdaInd>("udp_data", 10, [node](const v2x_stack::msg::CohdaInd::SharedPtr msg) {node->udpCallback(msg);});
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Create Subs");
    rclcpp::spin(node);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Spin Node");
    rclcpp::shutdown();

    return 0;
}

//} // namespace v2x_stack_btp

// Register the node as a component
// RCLCPP_COMPONENTS_REGISTER_NODE(v2x_stack_btp::BtpPublisher)
