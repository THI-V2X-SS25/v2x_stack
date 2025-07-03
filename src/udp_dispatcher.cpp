#include "udp_dispatcher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include "udp_msgs/msg/udp_packet.hpp"


namespace v2x_stack_btp
{
UDPdispatcher::UDPdispatcher(const rclcpp::NodeOptions &options)
    : Node("udp_publisher", options)
{
    // parameter's default values
    this->declare_parameter<std::string>("originating_ip", "172.16.2.1");
    this->declare_parameter<int>("originating_port", 4400);
    this->declare_parameter<std::string>("destination_ip", "172.16.2.1");
    this->declare_parameter<int>("destination_port", 4401);

    // Get parameter values from config.yml
    this->get_parameter("originating_ip", originatingIp);
    this->get_parameter("originating_port", originatingPort);
    this->get_parameter("destination_ip", destinationIP);
    this->get_parameter("destination_port", destinationPort);

    RCLCPP_INFO(this->get_logger(), "UDP Dispatcher receiving on IP: %s, Port: %d", originatingIp.c_str(), originatingPort);
    RCLCPP_INFO(this->get_logger(), "UDP Dispatcher sending to IP: %s, Port: %d", destinationIP.c_str(), destinationPort);
    
    //publihser to work with ETSI_PKG -> sensds pure udp package
    publisher = this->create_publisher<udp_msgs::msg::UdpPacket>("converter/udp/in", 10);

    //Node and publhiser for THI Development -> sends cohda converted udp package
    node_ = std::make_shared<rclcpp::Node>("udp_publisher_node");
    publisher_ = node_->create_publisher<v2x_stack_btp::msg::CohdaInd>("udp_data", 10);

    //Create Sender Socket
    send_sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (send_sockfd_ < 0) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Error creating UDP send socket");
        return;
    } else {
        RCLCPP_INFO(this->get_logger(), "Successfully created sender socket");
    }

    // Create subscriber for outgoing UDP packets
    udp_send_subscriber_ = this->create_subscription<udp_msgs::msg::UdpPacket>("/etsi_its_conversion/udp/out", 20,std::bind(&UDPdispatcher::createCohdaBTP, this, std::placeholders::_1));

    // Init sockets
    //initialize();    
}

void UDPdispatcher::initialize()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "UDP Dispatcher initialized");
    
    // Create receiving socket
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Error creating UDP socket");
        return;
    }

    struct sockaddr_in host_addr, sender_addr;
    host_addr.sin_family = AF_INET;
    host_addr.sin_port = htons(originatingPort);  // host_port
    host_addr.sin_addr.s_addr = INADDR_ANY;
    
    if (bind(sockfd, (struct sockaddr *)&host_addr, sizeof(host_addr)) < 0) {
        close(sockfd);
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Error binding socket");
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialized and starting to receive UDP packets");

    char buffer[1024];
    socklen_t addr_len = sizeof(sender_addr);

    while (true) {
        ssize_t recv_len = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&sender_addr, &addr_len);
        if (recv_len < 0) {
            RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Error receiving data");
            close(sockfd);
            break;
        }

        if (sender_addr.sin_addr.s_addr == inet_addr(originatingIp.c_str())) {
            if (recv_len < sizeof(tUDPBTPMsgType) + sizeof(tUDPBTPDataIndHdr)) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Received packet too short for BTP header");
                continue;
            }
            tUDPBTPDataIndMsg *udpPackage = reinterpret_cast<tUDPBTPDataIndMsg *>(buffer);
            publish(udpPackage);

            udp_msgs::msg::UdpPacket ros_udp_msg;
            ros_udp_msg.address = originatingIp.c_str();
            //ros_udp_msg.port = "4401";
            ros_udp_msg.data.assign(buffer, buffer + recv_len);
            publisher->publish(ros_udp_msg);
            
        }
    }
 
}

void UDPdispatcher::publish(const tUDPBTPDataIndMsg *ind)
{
    //v2x_stack::msg::CohdaInd ccu_ind;
    auto ccu_ind = boost::make_shared<v2x_stack_btp::msg::CohdaInd>();

    ccu_ind->type.version = ind->Type.Version;
    ccu_ind->type.msg_id = ind->Type.MsgID;
    ccu_ind->type.msg_length = ntohs(ind->Type.MsgLen);

    ccu_ind->header.btp_type = ind->Hdr.BTPType;
    ccu_ind->header.pkt_transport = ind->Hdr.PktTransport;
    ccu_ind->header.traffic_class = ind->Hdr.TrafficClass;
    ccu_ind->header.max_pkt_life_time = ind->Hdr.MaxPktLifetime;
    ccu_ind->header.dest_port = ntohs(ind->Hdr.DestPort);
   
    ccu_ind->header.dest_info = ind->Hdr.DestInfo;
    
    int btpMsgSize = ntohs(ind->Type.MsgLen);
    ccu_ind->payload.resize(btpMsgSize);

    std::copy(ind->Payload, ind->Payload + btpMsgSize, ccu_ind->payload.begin());

    

    publisher_->publish(*ccu_ind);
}

void UDPdispatcher::createCohdaBTP(const udp_msgs::msg::UdpPacket::SharedPtr udp)
{
    const std::vector<uint8_t>& btp = udp->data;

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


    //RCLCPP_INFO(this->get_logger(), "Hdr Size: %zu", (uper_payload_size + sizeof(tUDPBTPDataReqHdr)));

    //auto ros_udp_msg = udp_msgs::msg::UdpPacket{};
    //ros_udp_msg.address = "172.16.2.1";
    //ros_udp_msg.data = buffer;
    //publisher->publish(ros_udp_msg);
    send_udp_packet(buffer);

}

bool UDPdispatcher::send_udp_packet(const std::vector<uint8_t>& data)
{   
    //create sender address
    struct sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(destinationPort);  // dest port
    inet_pton(AF_INET, destinationIP.c_str(), &dest_addr.sin_addr);

    RCLCPP_INFO(this->get_logger(), "Entering SEND UDP Packet");

    ssize_t sent_bytes = sendto(send_sockfd_, data.data(), data.size(), 0,
                                (sockaddr*)&dest_addr, sizeof(dest_addr));

    if (sent_bytes != static_cast<ssize_t>(data.size()))
    {
        RCLCPP_ERROR(this->get_logger(), "sendto failed: %s", strerror(errno));
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Sent UDP packet to %s:%d (%ld bytes)",
                destinationIP.c_str(), destinationPort, sent_bytes);

    return true;
}

} // namespace v2x_stack_btp

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);    
    auto node = std::make_shared<v2x_stack_btp::UDPdispatcher>(rclcpp::NodeOptions{});

    // start udp receive frame
    std::thread recv_thread([&]() {node->initialize();});

    rclcpp::spin(node);   
    recv_thread.join();
    rclcpp::shutdown();

    return 0;
}

//RCLCPP_COMPONENTS_REGISTER_NODE(v2x_stack::UDPdispatcher)