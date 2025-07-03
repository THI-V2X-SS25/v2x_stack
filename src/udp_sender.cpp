#include "udp_sender.hpp"

#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>

namespace v2x_stack_btp
{

UDPSender::UDPSender(const rclcpp::NodeOptions &options)
    : Node("udp_sender_node", options)
{
    // Declare parameters with defaults
    this->declare_parameter<std::string>("destination_ip", "172.16.2.1");
    this->declare_parameter<int>("destination_port", 4401);

    // Get parameters
    this->get_parameter("destination_ip", destination_ip_);
    this->get_parameter("destination_port", destination_port_);

    // Create UDP socket
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0)
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to create UDP socket");
        throw std::runtime_error("UDP socket creation failed");
    }

    // Setup destination address struct
    memset(&dest_addr_, 0, sizeof(dest_addr_));
    dest_addr_.sin_family = AF_INET;
    dest_addr_.sin_port = htons(destination_port_);
    dest_addr_.sin_addr.s_addr = inet_addr(destination_ip_.c_str());

    RCLCPP_INFO(this->get_logger(), "UDP Sender initialized to send to %s:%d",
                destination_ip_.c_str(), destination_port_);

    // Create subscription to udp packet topic
    udp_subscriber_ = this->create_subscription<udp_msgs::msg::UdpPacket>(
        "etsi_its_conversion/udp/out", 10,
        std::bind(&UDPSender::udp_callback, this, std::placeholders::_1));
}

UDPSender::~UDPSender()
{
    close(sockfd_);
}

void UDPSender::udp_callback(const udp_msgs::msg::UdpPacket::SharedPtr msg)
{
    // IMPORTANT: Change 'data' to the actual byte array field name in your UdpPacket message!
    RCLCPP_INFO(this->get_logger(), "Received message with %zu bytes to send via UDP", msg->data.size());

    bool success = send_packet(msg->data.data(), msg->data.size());
    if (!success)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to send UDP packet");
    }
}

bool UDPSender::send_packet(const uint8_t *data, size_t size)
{
    RCLCPP_INFO(this->get_logger(), "Sending UDP packet...");

    ssize_t sent_bytes = sendto(sockfd_, data, size, 0,
                                reinterpret_cast<struct sockaddr *>(&dest_addr_), sizeof(dest_addr_));

    if (sent_bytes != static_cast<ssize_t>(size))
    {
        RCLCPP_ERROR(this->get_logger(), "sendto failed: %s", strerror(errno));
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Sent UDP packet to %s:%d (%ld bytes)",
                destination_ip_.c_str(), destination_port_, sent_bytes);

    return true;
}


}  // namespace v2x_stack_btp

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<v2x_stack_btp::UDPSender>(rclcpp::NodeOptions{});
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}