#ifndef UDP_SENDER_HPP_
#define UDP_SENDER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <udp_msgs/msg/udp_packet.hpp>

#include <string>
#include <cstdint>
#include <sys/socket.h>
#include <netinet/in.h>

namespace v2x_stack_btp
{

class UDPSender : public rclcpp::Node
{
public:
    explicit UDPSender(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~UDPSender();

private:
    void udp_callback(const udp_msgs::msg::UdpPacket::SharedPtr msg);
    bool send_packet(const uint8_t *data, size_t size);

    // UDP socket
    int sockfd_;
    struct sockaddr_in dest_addr_;

    // Destination IP and port
    std::string destination_ip_;
    int destination_port_;

    // ROS subscriber
    rclcpp::Subscription<udp_msgs::msg::UdpPacket>::SharedPtr udp_subscriber_;
};

}  // namespace v2x_stack_btp

#endif  // UDP_SENDER_HPP_