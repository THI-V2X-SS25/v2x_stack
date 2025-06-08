#include <boost/make_shared.hpp>
#include <etsi_its_mcm_thi_prima_msgs/msg/mcm.hpp>
#include <etsi_its_mcm_thi_prima_coding/asn_MCM.h>
#include <etsi_its_mcm_thi_prima_msgs/msg/path_point.hpp>

namespace etsi_its_messages_btp
{

namespace
{

inline uint8_t reverse_byte(uint8_t byte)
{
    byte = (byte & 0xf0) >> 4 | (byte & 0x0f) << 4;
    byte = (byte & 0xcc) >> 2 | (byte & 0x33) << 2;
    byte = (byte & 0xaa) >> 1 | (byte & 0x55) << 1;
    return byte;
}

} // namespace


/**
    Convert ASN1 asn_MCM.h (etsi_its_messages/etsi_its_coding/etsi_its_mcm_thi_prima_coding/include/etsi_its_mcm_thi_prima_coding/asn_MCM.h) to ROS2 mcm.msg (etsi_its_messages/etsi_its_msgs/etsi_its_mcm_thi_prima_msgs/msg/MCM.msg)
    based on the example of ca_message.cpp (v2x_stack/src/ca_message.cpp) with CAM.msg (ros2_etsi_its_msgs/msg/CAM.msg) and CAM.h (v2x_stack/extern/vanetza/vanetza/asn1/its/CAM.h)
 */
boost::shared_ptr<etsi_its_mcm_thi_prima_msgs::msg::MCM> convertMCM(const asn_MCM* asn1, std::string* error_msg)
{
  auto msg = boost::make_shared<etsi_its_mcm_thi_prima_msgs::msg::MCM>();

  //ItsPduHeader
  msg->header.protocol_version = asn1->header.protocolVersion;
  msg->header.message_id = asn1->header.messageID;
  msg->header.station_id.value = asn1->header.stationID;        // copy on value of stationID, not its self

  //ManeuverCoordinationMessage
  msg->mcm.generation_delta_time.value = asn1->mcm.generationDeltaTime;

  //ManeuverCoordinationMessage --> Parameters --> Basic Container
  auto& basic_container = msg->mcm.mcm_parameters.basic_container_mcm; //ROS2
  auto& basicContainerMCM = asn1->mcm.mcmParameters.basicContainerMCM; //ASN1
  basic_container.station_type.value = basicContainerMCM.stationType;
  basic_container.reference_position.latitude.value = basicContainerMCM.referencePosition.latitude;
  basic_container.reference_position.longitude.value = basicContainerMCM.referencePosition.longitude;
  basic_container.reference_position.position_confidence_ellipse.semi_major_confidence.value = basicContainerMCM.referencePosition.positionConfidenceEllipse.semiMajorConfidence;
  basic_container.reference_position.position_confidence_ellipse.semi_minor_confidence.value = basicContainerMCM.referencePosition.positionConfidenceEllipse.semiMinorConfidence;
  basic_container.reference_position.position_confidence_ellipse.semi_major_orientation.value = basicContainerMCM.referencePosition.positionConfidenceEllipse.semiMajorOrientation;
  basic_container.reference_position.altitude.altitude_value.value = basicContainerMCM.referencePosition.altitude.altitudeValue;
  basic_container.reference_position.altitude.altitude_confidence.value = basicContainerMCM.referencePosition.altitude.altitudeConfidence;

  //ManeuverCoordinationMessage --> Parameters --> intentionSharingContainer
  auto& intention_sharing_container = msg->mcm.mcm_parameters.intention_sharing_container; //ROS2
  auto& intentionSharingContainer = asn1->mcm.mcmParameters.intentionSharingContainer; //ASN1

  for (int i = 0; i < intentionSharingContainer.plannedTrajectory.list.count; ++i)
  {
      auto& asn1_path_point = asn1->mcm.mcmParameters.intentionSharingContainer.plannedTrajectory.list.array[i];
      etsi_its_mcm_thi_prima_msgs::msg::TrajectoryPointMCM path_point;

      path_point.delta_longitudinal_position.value = asn1_path_point->deltaLongitudinalPosition;
      path_point.delta_lateral_position.value = asn1_path_point->deltaLateralPosition;
      path_point.delta_heading.value = asn1_path_point->deltaHeading;

      path_point.delta_time.value = 0;
      if (asn1_path_point->deltaTime)
      {
          path_point.delta_time.value = asn1_path_point->deltaTime;
      }

      msg->mcm.mcm_parameters.intention_sharing_container.planned_trajectory.array.push_back(path_point);
  }


  intention_sharing_container.heading.heading_value.value = intentionSharingContainer.heading.headingValue;
  intention_sharing_container.heading.heading_confidence.value = intentionSharingContainer.heading.headingConfidence;

  intention_sharing_container.speed.speed_value.value = intentionSharingContainer.speed.speedValue;
  intention_sharing_container.speed.speed_confidence.value = intentionSharingContainer.speed.speedConfidence;

  intention_sharing_container.drive_direction.value = intentionSharingContainer.driveDirection;

  intention_sharing_container.vehicle_length.vehicle_length_value.value = intentionSharingContainer.vehicleLength.vehicleLengthValue;
  intention_sharing_container.vehicle_length.vehicle_length_confidence_indication.value = intentionSharingContainer.vehicleLength.vehicleLengthConfidenceIndication;

  intention_sharing_container.vehicle_width.value = intentionSharingContainer.vehicleWidth;

  intention_sharing_container.vehicle_automation_level.value = intentionSharingContainer.vehicleAutomationLevel;


  return msg;
}
}
