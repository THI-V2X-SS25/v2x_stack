#include <ros_etsi_its_msgs/msg/cam.hpp>
#include <etsi_its_cam/asn_CAM.h> // Beispielpfad, passe ggf. an
#include <cstdlib>
#include <cstring>

namespace v2x_stack_btp
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

asn_CAM_t* convertCam(const ros_etsi_its_msgs::msg::CAM::ConstSharedPtr& ptr)
{
    auto* msg = static_cast<asn_CAM_t*>(calloc(1, sizeof(asn_CAM_t)));

    // === Header ===
    msg->header.protocolVersion = ptr->its_header.protocol_version;
    msg->header.messageID = ptr->its_header.message_id;
    msg->header.stationID = ptr->its_header.station_id;

    // === CAM Payload ===
    msg->cam.generationDeltaTime = ptr->generation_delta_time;

    auto& basic = msg->cam.camParameters.basicContainer;
    basic.stationType = ptr->station_type.value;
    basic.referencePosition.altitude.altitudeValue = ptr->reference_position.altitude.value;
    basic.referencePosition.altitude.altitudeConfidence = ptr->reference_position.altitude.confidence;
    basic.referencePosition.latitude = ptr->reference_position.latitude;
    basic.referencePosition.longitude = ptr->reference_position.longitude;
    basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence =
        ptr->reference_position.position_confidence.semi_major_confidence;
    basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence =
        ptr->reference_position.position_confidence.semi_minor_confidence;
    basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation =
        ptr->reference_position.position_confidence.semi_major_orientation;

    auto& hfc = msg->cam.camParameters.highFrequencyContainer;
    hfc.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;

    auto& bvc = hfc.choice.basicVehicleContainerHighFrequency;
    bvc.heading.headingValue = ptr->high_frequency_container.heading.value;
    bvc.heading.headingConfidence = ptr->high_frequency_container.heading.confidence;
    bvc.speed.speedValue = ptr->high_frequency_container.speed.value;
    bvc.speed.speedConfidence = ptr->high_frequency_container.speed.confidence;
    bvc.driveDirection = ptr->high_frequency_container.drive_direction.value;
    bvc.longitudinalAcceleration.longitudinalAccelerationValue = ptr->high_frequency_container.longitudinal_acceleration.value;
    bvc.longitudinalAcceleration.longitudinalAccelerationConfidence = ptr->high_frequency_container.longitudinal_acceleration.confidence;
    bvc.curvature.curvatureValue = ptr->high_frequency_container.curvature.value;
    bvc.curvature.curvatureConfidence = ptr->high_frequency_container.curvature.confidence;
    bvc.curvatureCalculationMode = ptr->high_frequency_container.curvature_calculation_mode.value;
    bvc.yawRate.yawRateValue = ptr->high_frequency_container.yaw_rate.value;
    bvc.yawRate.yawRateConfidence = ptr->high_frequency_container.yaw_rate.confidence;
    bvc.vehicleLength.vehicleLengthValue = ptr->high_frequency_container.vehicle_length.value;
    bvc.vehicleLength.vehicleLengthConfidenceIndication = ptr->high_frequency_container.vehicle_length.confidence_indication;
    bvc.vehicleWidth = ptr->high_frequency_container.vehicle_width.value;

    if (ptr->high_frequency_container.has_acceleration_control)
    {
        bvc.accelerationControl = static_cast<AccelerationControl_t*>(calloc(1, sizeof(AccelerationControl_t)));
        bvc.accelerationControl->buf = static_cast<uint8_t*>(calloc(1, sizeof(uint8_t)));
        bvc.accelerationControl->size = 1;
        bvc.accelerationControl->buf[0] = reverse_byte(ptr->high_frequency_container.acceleration_control.value);
    }

    if (ptr->has_low_frequency_container)
    {
        msg->cam.camParameters.lowFrequencyContainer =
            static_cast<LowFrequencyContainer_t*>(calloc(1, sizeof(LowFrequencyContainer_t)));
        auto& bvcl = msg->cam.camParameters.lowFrequencyContainer->choice.basicVehicleContainerLowFrequency;
        msg->cam.camParameters.lowFrequencyContainer->present = LowFrequencyContainer_PR_basicVehicleContainerLowFrequency;

        bvcl.vehicleRole = ptr->low_frequency_container.vehicle_role.value;
        bvcl.exteriorLights.buf = static_cast<uint8_t*>(calloc(1, sizeof(uint8_t)));
        bvcl.exteriorLights.size = 1;
        bvcl.exteriorLights.buf[0] = reverse_byte(ptr->low_frequency_container.exterior_lights.value);

        for (const auto& point : ptr->low_frequency_container.path_history.points)
        {
            auto* path_point = static_cast<PathPoint_t*>(calloc(1, sizeof(PathPoint_t)));

            if (point.path_delta_time.value != ros_etsi_its_msgs::msg::PathDeltaTime::UNAVAILABLE)
            {
                path_point->pathDeltaTime = static_cast<PathDeltaTime_t*>(calloc(1, sizeof(PathDeltaTime_t)));
                *path_point->pathDeltaTime = point.path_delta_time.value;
            }

            path_point->pathPosition.deltaAltitude = point.path_position.delta_altitude;
            path_point->pathPosition.deltaLatitude = point.path_position.delta_latitude;
            path_point->pathPosition.deltaLongitude = point.path_position.delta_longitude;

            ASN_SEQUENCE_ADD(&bvcl.pathHistory.list, path_point);
        }
    }

    return msg;
}

} // namespace v2x_stack_btp

