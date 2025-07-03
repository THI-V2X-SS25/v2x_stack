#include "udp_sender_converter.hpp"

#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>

namespace v2x_stack_btp
{

UDPSenderConverter::UDPSenderConverter(const rclcpp::NodeOptions &options)
    : Node("udp_sender_converter_node", options)
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
        std::bind(&UDPSenderConverter::udp_converter_callback, this, std::placeholders::_1));
    
    //publihser to work with ETSI_PKG -> sensds pure udp package
    publisher = this->create_publisher<udp_msgs::msg::UdpPacket>("etsi_its_conversion/udp/in", 10);
}

UDPSenderConverter::~UDPSenderConverter()
{
    close(sockfd_);
}

void UDPSenderConverter::udp_converter_callback(const udp_msgs::msg::UdpPacket::SharedPtr msg)
{
    // IMPORTANT: Change 'data' to the actual byte array field name in your UdpPacket message!
    //RCLCPP_INFO(this->get_logger(), "Received message with %zu bytes to send via UDP", msg->data.size());
    const std::vector<uint8_t>& rawBTP = msg->data;

    createCohdaBTP(rawBTP);
}

void UDPSenderConverter::createCohdaBTP(const std::vector<uint8_t>& btp)
{
    
    uint16_t dest_port = (btp[0] << 8) | btp[1];
    uint16_t dest_port_info = (btp[2] << 8) | btp[3];
    
    const uint8_t* payload_data = btp.data() + 4;
    size_t uper_payload_size = btp.size();
    size_t payload_size = btp.size() - 4;
    size_t total_size = sizeof(tUDPBTPMsgType) + sizeof(tUDPBTPDataReqHdr) + payload_size;

    std::vector<uint8_t> buffer(total_size);

    // Pointer to structs inside buffer
    tUDPBTPDataReqMsg* msg = reinterpret_cast<tUDPBTPDataReqMsg*>(buffer.data());

    // Fill MsgType
    msg->Type.Version = 4;
    msg->Type.MsgID = 0;
    msg->Type.MsgLen = htons(payload_size + sizeof(tUDPBTPDataReqHdr)); 

    // Fill Hdr
    msg->Hdr.BTPType = 2;
    msg->Hdr.PktTransport = 7;
    msg->Hdr.TrafficClass = 0x02;
    msg->Hdr.MaxPktLifetime = 0;
    msg->Hdr.DestPort = htons(dest_port);
    msg->Hdr.DestInfo = htons(0);
    msg->Hdr.SrcPort = htons(0);

    msg->Hdr.Location.GN_ADDR_as_64 = htonl(0);
    msg->Hdr.Location.Unused = htonl(0);

    // GBC/GAC/GUC Destination Information
    msg->Hdr.Area.Latitude = htonl(0);
    msg->Hdr.Area.Longitude = htonl(0);
    msg->Hdr.Area.Distance_a = htons(0);
    msg->Hdr.Area.Distance_b = htons(0);
    msg->Hdr.Area.Angle = htons(0);
    msg->Hdr.Area.Shape = 0;
    msg->Hdr.Area.Unused = 0;

    msg->Hdr.CommProfile = 0;
    msg->Hdr.RepeatInterval = 0;
    msg->Hdr.SecProfile = 0;
    msg->Hdr.SSPLen = 0;
    msg->Hdr.AID = htonl(0x24);
    std::memset(msg->Hdr.SSPBits, 0, sizeof(msg->Hdr.SSPBits));
    msg->Hdr.SSPBits[0] = 0x00;
    msg->Hdr.Length = htons(payload_size);

    // Copy ASN.1 payload
    //std::memcpy(msg->Payload, btp.data(), uper_payload_size);

    //std::copy(btp.begin(), btp.end(), msg->Payload);
    std::memcpy(msg->Payload, payload_data, payload_size);


    RCLCPP_INFO(this->get_logger(), "Hdr Size: %zu", (uper_payload_size + sizeof(tUDPBTPDataReqHdr)));

    auto ros_udp_msg = udp_msgs::msg::UdpPacket{};
    ros_udp_msg.address = "172.16.2.1";
    ros_udp_msg.data = buffer;
    publisher->publish(ros_udp_msg);
    send_udp_packet(buffer);

}

bool UDPSenderConverter::send_udp_packet(const std::vector<uint8_t>& data)
{
    RCLCPP_INFO(this->get_logger(), "Sending UDP packet...");

    ssize_t sent_bytes = sendto(sockfd_, data.data(), data.size(), 0,
                                (sockaddr*)&dest_addr_, sizeof(dest_addr_));

    if (sent_bytes != static_cast<ssize_t>(data.size()))
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

    auto node = std::make_shared<v2x_stack_btp::UDPSenderConverter>(rclcpp::NodeOptions{});
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}