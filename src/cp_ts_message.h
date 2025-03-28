#pragma once

#include <boost/shared_ptr.hpp>
#include <etsi_its_cpm_ts_msgs/msg/collective_perception_message.hpp>
#include <string>

// forward declaration
namespace vanetza { namespace asn1 { namespace r2 {class Cpm;} } }

namespace v2x_stack_btp
{

/**
 * Convert ASN.1 data structure to ROS etsi_its_msg CPM
 * \param cpm ASN.1 data structure
 * \param msg optional error string
 * \return converted CPM (or nullptr on error)
 */
boost::shared_ptr<etsi_its_cpm_ts_msgs::msg::CollectivePerceptionMessage> convertCpm(const vanetza::asn1::r2::Cpm& cpm, std::string* msg = nullptr);

/**
 * Convert ROS etsi_its_msg CPM to ASN.1 data structure
 * \param ptr etsi_its_msgs CPM
 * \return converted CPM
 */
//vanetza::asn1::Com convertCpm(ros_etsi_its_msgs::msg::CPM::ConstSharedPtr ptr);

} // namespace v2x_stack_btp