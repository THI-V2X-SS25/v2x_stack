#ifndef BTP_SERVICE_HPP
#define BTP_SERVICE_HPP

#include "rclcpp/rclcpp.hpp"
#include "v2x_stack_btp/msg/cohda_ind.hpp"
#include "v2x_stack_btp/srv/btp_data.hpp"
#include "v2x_stack_btp/msg/btp_data_indication.hpp"
#include <vanetza/btp/data_indication.hpp>
#include <vanetza/geonet/transport_interface.hpp>


namespace v2x_stack_btp
{

class BtpPublisher : public rclcpp::Node
{
public:
    explicit BtpPublisher(const rclcpp::NodeOptions & options);


    void udpCallback(const v2x_stack_btp::msg::CohdaInd::SharedPtr msg);
    bool serviceCallback(const std::shared_ptr<srv::BtpData::Request> request,
                                std::shared_ptr<srv::BtpData::Response> response);

    void publish (const v2x_stack_btp::msg::CohdaInd::SharedPtr packet);                                
    
    rclcpp::Subscription<v2x_stack_btp::msg::CohdaInd>::SharedPtr subscription_;
    rclcpp::Service<srv::BtpData>::SharedPtr service_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<msg::BtpDataIndication>::SharedPtr publisher_;
    
};

} // namespace v2x_stack_btp

#endif // BTP_SERVICE_HPP
