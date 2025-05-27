#include "mcm_converter.hpp"
#include <cstring>

namespace v2x_stack_mcm
{

bool convertMcmRosToAsn1(const ros_etsi_its_msgs::msg::MCM& ros_msg, ManeuverCoordinationMessage_t& asn1_msg)
{
    asn1_msg.header.protocolVersion = ros_msg.header.protocol_version;
    asn1_msg.header.messageID = ros_msg.header.message_id;
    asn1_msg.header.stationID = ros_msg.header.station_id;

    asn1_msg.senderId = ros_msg.sender_id;
    asn1_msg.recipientId = ros_msg.recipient_id;

    asn1_msg.maneuver.type = ros_msg.maneuver_type.value;
    asn1_msg.maneuver.startTime = ros_msg.maneuver.start_time;
    asn1_msg.maneuver.duration = ros_msg.maneuver.duration;

    ASN_STRUCT_RESET(asn_DEF_ManeuverCoordinationMessage, &asn1_msg);
    for (const auto& tp : ros_msg.trajectory.points) {
        TrajectoryPoint_t* asn_tp = (TrajectoryPoint_t*)calloc(1, sizeof(TrajectoryPoint_t));
        asn_tp->position.x = tp.position.x;
        asn_tp->position.y = tp.position.y;
        asn_tp->position.z = tp.position.z;
        asn_tp->timestamp = tp.timestamp;

        ASN_SEQUENCE_ADD(&asn1_msg.trajectory, asn_tp);
    }

    return true;
}

} 
