#include <boost/make_shared.hpp>
#include <etsi_its_mcm_thi_prima_msgs/msg/mcm.hpp>
#include <etsi_its_mcm_thi_prima_coding/asn_MCM.h>
#include <etsi_its_mcm_thi_prima_msgs/msg/path_point.hpp>

namespace etsi_its_messages_btp
{
boost::shared_ptr<etsi_its_mcm_thi_prima_msgs::msg::MCM> convertMCM(const asn_MCM* asn1, std::string* error_msg);
}