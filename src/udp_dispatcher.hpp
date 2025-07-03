#ifndef UDP_DISPATCHER_H_
#define UDP_DISPATCHER_H_

#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include "ccu_udp_api.h"
#include "v2x_stack_btp/msg/cohda_ind.hpp"
#include "udp_msgs/msg/udp_packet.hpp"

namespace v2x_stack_btp
{
class UDPdispatcher : public rclcpp::Node
{
public:
    UDPdispatcher(const rclcpp::NodeOptions &options);
    //virtual void requestTransmission();
    //void indicate();
    void publish(const tUDPBTPDataIndMsg* ind);
    void initialize();

private:        
    
    // Method to send UDP packet from ROS topic callback
    bool send_udp_packet(const std::vector<uint8_t>& data);
    void createCohdaBTP(const udp_msgs::msg::UdpPacket::SharedPtr udp);
    
    // UDP socket descriptors and destination address    
    int recv_sockfd_;
    int send_sockfd_;
    struct sockaddr_in host_addr, ccu_addr;
    struct sockaddr_in dest_addr_;  // for sending
    std::string ccu_ip;
    uint16_t ccu_port;
    uint16_t host_port;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<v2x_stack_btp::msg::CohdaInd>::SharedPtr publisher_;
    rclcpp::Publisher<udp_msgs::msg::UdpPacket>::SharedPtr publisher;
    rclcpp::Subscription<udp_msgs::msg::UdpPacket>::SharedPtr udp_send_subscriber_;  // the new to send 

    std::string originatingIp;    
    int originatingPort;
    std::string destinationIP;
    int destinationPort;
    
    tUDPBTPDataIndMsg receivedPackage;
};

} // namespace v2x_stack_btp 
#endif // UDP_DISPATCHER_H_