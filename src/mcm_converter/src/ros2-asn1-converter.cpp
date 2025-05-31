#include <etsi_its_msgs/msg/mcm.hpp>
#include <etsi_its_mcm_thi_prima_coding/asn_MCM.h>
#include <memory>
#include <cstring>

namespace etsi_its_messages_btp
{

bool convertMCMToASN1(const etsi_its_msgs::msg::MCM& ros_msg, asn_MCM_t& asn1_msg, std::string* error_msg)
{
    // --- ItsPduHeader ---
    asn1_msg.header.protocolVersion = ros_msg.header.protocol_version;
    asn1_msg.header.messageID = ros_msg.header.message_id;
    asn1_msg.header.stationID = ros_msg.header.station_id;

    // mcm
    asn1_msg.mcm.generationDeltaTime = ros_msg.mcm.generation_delta_time;

    auto& basic_container = ros_msg.mcm.mcm_parameters.basic_container_mcm;
    auto& basic_asn = asn1_msg.mcm.mcmParameters.basicContainerMCM;

    basic_asn.stationType = basic_container.station_type.value;
    basic_asn.referencePosition.latitude = basic_container.reference_position.latitude.value;
    basic_asn.referencePosition.longitude = basic_container.reference_position.longitude.value;
    basic_asn.referencePosition.positionConfidenceEllipse.semiMajorConfidence =
        basic_container.reference_position.position_confidence_ellipse.semi_major_confidence.value;
    basic_asn.referencePosition.positionConfidenceEllipse.semiMinorConfidence =
        basic_container.reference_position.position_confidence_ellipse.semi_minor_confidence.value;
    basic_asn.referencePosition.positionConfidenceEllipse.semiMajorOrientation =
        basic_container.reference_position.position_confidence_ellipse.semi_major_orientation.value;
    basic_asn.referencePosition.altitudeValue =
        basic_container.reference_position.altitude_value.value;
    basic_asn.referencePosition.altitudeConfidence =
        basic_container.reference_position.altitude_confidence.value;

    // --- IntentionSharingContainer ---
    auto& isc_ros = ros_msg.mcm.mcm_parameters.intention_sharing_container;
    auto& isc_asn = asn1_msg.mcm.mcmParameters.intentionSharingContainer;

    // Allocate memory for the trajectory points
    int point_count = static_cast<int>(isc_ros.planned_trajectory.size());
    isc_asn.plannedTrajectory.list.count = point_count;
    isc_asn.plannedTrajectory.list.size = point_count;
    isc_asn.plannedTrajectory.list.array = (PathPoint_t**)calloc(point_count, sizeof(PathPoint_t*));

    for (int i = 0; i < point_count; ++i)
    {
        const auto& ros_point = isc_ros.planned_trajectory[i];
        PathPoint_t* asn_point = (PathPoint_t*)calloc(1, sizeof(PathPoint_t));
        if (!asn_point) {
            if (error_msg) *error_msg = "Memory allocation failed for plannedTrajectory element.";
            return false;
        }
        asn_point->deltaLongitudinalPosition.deltaLatitude = ros_point.delta_longitudinal_position.value;
        asn_point->deltaLateralPosition.deltaLongitude = ros_point.delta_lateral_position.value;
        asn_point->delta_heading.deltaHeading = ros_point.delta_heading.value;

        if (ros_point.path_delta_time.value != etsi_its_msgs::msg::PathDeltaTime::UNAVAILABLE) {
            asn_point->pathDeltaTime = (PathDeltaTime_t*)calloc(1, sizeof(PathDeltaTime_t));
            if (!asn_point->pathDeltaTime) {
                if (error_msg) *error_msg = "Memory allocation failed for pathDeltaTime.";
                return false;
            }
            *(asn_point->pathDeltaTime) = ros_point.path_delta_time.value;
        } else {
            asn_point->pathDeltaTime = nullptr;
        }

        isc_asn.plannedTrajectory.list.array[i] = asn_point;
    }

    isc_asn.heading.headingValue = isc_ros.heading.heading_value.value;
    isc_asn.heading.headingConfidence = isc_ros.heading.heading_confidence.value;
    isc_asn.speed.speedValue = isc_ros.speed.speed_value.value;
    isc_asn.speed.speedConfidence = isc_ros.speed.speed_confidence.value;
    isc_asn.driveDirection = isc_ros.drive_direction.value;
    isc_asn.vehicleLength.vehicleLengthValue = isc_ros.vehicle_length.vehicle_length_value.value;
    isc_asn.vehicleLength.vehicleLengthConfidenceIndication =
        isc_ros.vehicle_length.vehicle_length_confidence_indication.value;
    isc_asn.vehicleWidth = isc_ros.vehicle_width.value;
    isc_asn.vehicleAutomationLevel = isc_ros.vehicle_automation_level.value;
    isc_asn.lanePosition = isc_ros.lane_position.value;

    return true;
}
} 
