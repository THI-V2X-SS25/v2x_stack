#ifndef UDP_SENDER_CONVERTER_HPP_
#define UDP_SENDER_CONVERTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <udp_msgs/msg/udp_packet.hpp>
#include "ccu_udp_api.h"
#include <string>
#include <cstdint>
#include <sys/socket.h>
#include <netinet/in.h>

namespace v2x_stack_btp
{

class UDPSenderConverter : public rclcpp::Node
{
public:
    explicit UDPSenderConverter(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~UDPSenderConverter();

private:
    void udp_converter_callback(const udp_msgs::msg::UdpPacket::SharedPtr msg);
    void createCohdaBTP(const std::vector<uint8_t>& btp);
    bool send_udp_packet(const std::vector<uint8_t>& data);

    // UDP socket
    int sockfd_;
    struct sockaddr_in dest_addr_;

    // Destination IP and port
    std::string destination_ip_;
    int destination_port_;

    // ROS subscriber
    rclcpp::Subscription<udp_msgs::msg::UdpPacket>::SharedPtr udp_subscriber_;
    rclcpp::Publisher<udp_msgs::msg::UdpPacket>::SharedPtr publisher;
};

}  // namespace v2x_stack_btp

#endif  // UDP_SENDER_CONVERTER_HPP_