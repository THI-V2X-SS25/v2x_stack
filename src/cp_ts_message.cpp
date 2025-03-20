#include "cp_ts_message.h"
#include <boost/make_shared.hpp>
#include <etsi_its_cpm_ts_msgs/msg/collective_perception_message.hpp>
#include <vanetza/asn1/cpm.hpp>
#include <algorithm>

namespace v2x_stack_btp
{


inline uint8_t reverse_byte(uint8_t byte)
{
    byte = (byte & 0xf0) >> 4 | (byte & 0x0f) << 4;
    byte = (byte & 0xcc) >> 2 | (byte & 0x33) << 2;
    byte = (byte & 0xaa) >> 1 | (byte & 0x55) << 1;
    return byte;
}

boost::shared_ptr<etsi_its_cpm_ts_msgs::msg::CollectivePerceptionMessage> convertCpm(const vanetza::asn1::r2::Cpm& asn1, std::string* error_msg)
{
    auto msg = boost::make_shared<etsi_its_cpm_ts_msgs::msg::CollectivePerceptionMessage>();

    // Header
    msg->header.protocol_version.value = asn1->header.protocolVersion;
    msg->header.message_id.value = asn1->header.messageId;
    msg->header.station_id.value = asn1->header.stationId;

    // Management Container
    asn_INTEGER2uint64(&asn1->payload.managementContainer.referenceTime, &msg->payload.management_container.reference_time.value);
    msg->payload.management_container.reference_position.altitude.altitude_value.value = asn1->payload.managementContainer.referencePosition.altitude.altitudeValue;
    msg->payload.management_container.reference_position.altitude.altitude_confidence.value = asn1->payload.managementContainer.referencePosition.altitude.altitudeConfidence;
    msg->payload.management_container.reference_position.latitude.value =  asn1->payload.managementContainer.referencePosition.latitude;
    msg->payload.management_container.reference_position.longitude.value = asn1->payload.managementContainer.referencePosition.longitude;
    msg->payload.management_container.reference_position.position_confidence_ellipse.semi_major_confidence.value = asn1->payload.managementContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence;
    msg->payload.management_container.reference_position.position_confidence_ellipse.semi_minor_confidence.value = asn1->payload.managementContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence;
    msg->payload.management_container.reference_position.position_confidence_ellipse.semi_major_orientation.value = asn1->payload.managementContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation;
    // TODO: Add optional MessageSegmentationInfo and MessageRateRange datafields

    if(asn1->payload.cpmContainers.list.size > 0)
    {
        for (long i = 0; i < asn1->payload.cpmContainers.list.count; ++i)
        {

            const Vanetza_ITS2_WrappedCpmContainer_t* asn1_wrapped_cont = asn1->payload.cpmContainers.list.array[i];
            etsi_its_cpm_ts_msgs::msg::WrappedCpmContainer wrapped_cont;

            wrapped_cont.container_id.value = asn1_wrapped_cont->containerId;

            switch (asn1_wrapped_cont->containerId)
            {
            // Originating Vehicle
            case 1:
                
                wrapped_cont.container_data_originating_vehicle_container.orientation_angle.value.value = asn1_wrapped_cont->containerData.choice.OriginatingVehicleContainer.orientationAngle.value;
                wrapped_cont.container_data_originating_vehicle_container.orientation_angle.confidence.value = asn1_wrapped_cont->containerData.choice.OriginatingVehicleContainer.orientationAngle.confidence;

                //TODO: optionals
                //
                // pitchAngle         CartesianAngle OPTIONAL,
                // rollAngle          CartesianAngle OPTIONAL,
                // trailerDataSet     TrailerDataSet OPTIONAL,

                break;
            
            // Originating RSU
            case 2:
                
                if(asn1_wrapped_cont->containerData.choice.OriginatingRsuContainer.mapReference->present){
                    wrapped_cont.container_data_originating_rsu_container.map_reference_is_present = true;
                    
                    switch (asn1_wrapped_cont->containerData.choice.OriginatingRsuContainer.mapReference->present)
                    {
                    case Vanetza_ITS2_MapReference_PR_NOTHING:
                        break;

                    case Vanetza_ITS2_MapReference_PR_roadsegment:
                        wrapped_cont.container_data_originating_rsu_container.map_reference.choice = Vanetza_ITS2_MapReference_PR_roadsegment;
                        wrapped_cont.container_data_originating_rsu_container.map_reference.roadsegment.id.value = asn1_wrapped_cont->containerData.choice.OriginatingRsuContainer.mapReference->choice.roadsegment.id;

                        if(asn1_wrapped_cont->containerData.choice.OriginatingRsuContainer.mapReference->choice.roadsegment.region){
                            wrapped_cont.container_data_originating_rsu_container.map_reference.roadsegment.region_is_present =  true;
                            wrapped_cont.container_data_originating_rsu_container.map_reference.roadsegment.region.value = static_cast<short unsigned int>(*asn1_wrapped_cont->containerData.choice.OriginatingRsuContainer.mapReference->choice.roadsegment.region);
                        }
                        break;

                    case Vanetza_ITS2_MapReference_PR_intersection:
                        wrapped_cont.container_data_originating_rsu_container.map_reference.choice = Vanetza_ITS2_MapReference_PR_intersection;
                        wrapped_cont.container_data_originating_rsu_container.map_reference.intersection.id.value = asn1_wrapped_cont->containerData.choice.OriginatingRsuContainer.mapReference->choice.intersection.id;

                        if(asn1_wrapped_cont->containerData.choice.OriginatingRsuContainer.mapReference->choice.intersection.region){
                            wrapped_cont.container_data_originating_rsu_container.map_reference.intersection.region_is_present =  true;
                            wrapped_cont.container_data_originating_rsu_container.map_reference.intersection.region.value = static_cast<short unsigned int>(*asn1_wrapped_cont->containerData.choice.OriginatingRsuContainer.mapReference->choice.intersection.region);
                        }

                        break;
                    
                    default:
                        break;
                    }
                }
                
                break;
            
            // Sensor Information 
            case 3:

                for(int j = 0; j < asn1_wrapped_cont->containerData.choice.SensorInformationContainer.list.count; j++)
                {
                    const Vanetza_ITS2_SensorInformation_t* asn1_sensor_info = asn1_wrapped_cont->containerData.choice.SensorInformationContainer.list.array[j];
                    etsi_its_cpm_ts_msgs::msg::SensorInformation sensor_info;
                    
                    sensor_info.sensor_id.value = asn1_sensor_info->sensorId;
                    sensor_info.sensor_type.value = asn1_sensor_info->sensorType;
                    sensor_info.shadowing_applies = asn1_sensor_info->shadowingApplies;

                    // TODO: optionals
                    //
                    // perceptionRegionShape		Shape OPTIONAL,
                    // perceptionRegionConfidence  ConfidenceLevel OPTIONAL,
                    
                    wrapped_cont.container_data_sensor_information_container.array.push_back(sensor_info);
                }  

                break;

            // Perception Region
            case 4:

                for(int j = 0; j < asn1_wrapped_cont->containerData.choice.PerceptionRegionContainer.list.count; j++)
                {
                    const Vanetza_ITS2_PerceptionRegion_t* asn1_perception_region = asn1_wrapped_cont->containerData.choice.PerceptionRegionContainer.list.array[j];
                    etsi_its_cpm_ts_msgs::msg::PerceptionRegion perception_region;

                    perception_region.measurement_delta_time.value = asn1_perception_region->measurementDeltaTime;
                    perception_region.perception_region_confidence.value = asn1_perception_region->perceptionRegionConfidence;

                    if(asn1_perception_region->perceptionRegionShape.present){

                        switch (asn1_perception_region->perceptionRegionShape.present)
                        {
                        case Vanetza_ITS2_Shape_PR_NOTHING:
                            break;

                        case Vanetza_ITS2_Shape_PR_rectangular:
                            perception_region.perception_region_shape.choice = Vanetza_ITS2_Shape_PR_rectangular;
                            perception_region.perception_region_shape.rectangular.semi_length.value = asn1_perception_region->perceptionRegionShape.choice.rectangular.semiLength;
                            perception_region.perception_region_shape.rectangular.semi_breadth.value = asn1_perception_region->perceptionRegionShape.choice.rectangular.semiBreadth;

                            // TODO: optionals
                            //
                            // centerPoint    CartesianPosition3d OPTIONAL,
                            // orientation    Wgs84AngleValue OPTIONAL,
                            // height         StandardLength12b OPTIONAL

                            break;

                        case Vanetza_ITS2_Shape_PR_circular:
                            perception_region.perception_region_shape.choice = Vanetza_ITS2_Shape_PR_circular;
                            perception_region.perception_region_shape.circular.radius.value = asn1_perception_region->perceptionRegionShape.choice.circular.radius;

                            // TODO: optionals
                            //
                            // shapeReferencePoint    CartesianPosition3d OPTIONAL,
                            // height                 StandardLength12b OPTIONAL

                            break;

                        case Vanetza_ITS2_Shape_PR_polygonal:
                            perception_region.perception_region_shape.choice = Vanetza_ITS2_Shape_PR_polygonal;

                            for(int k = 0; k <  asn1_perception_region->perceptionRegionShape.choice.polygonal.polygon.list.count; k++)
                            {
                                const Vanetza_ITS2_CartesianPosition3d_t* asn1_cartesian_pos3D = asn1_perception_region->perceptionRegionShape.choice.polygonal.polygon.list.array[k];
                                etsi_its_cpm_ts_msgs::msg::CartesianPosition3d cartesian_pos3D;

                                cartesian_pos3D.x_coordinate.value = asn1_cartesian_pos3D->xCoordinate;
                                cartesian_pos3D.y_coordinate.value = asn1_cartesian_pos3D->yCoordinate;

                                if(asn1_cartesian_pos3D->zCoordinate)
                                {
                                    cartesian_pos3D.z_coordinate_is_present = true;
                                    cartesian_pos3D.z_coordinate.value = static_cast<short int>(*asn1_cartesian_pos3D->zCoordinate);
                                }

                                perception_region.perception_region_shape.polygonal.polygon.array.push_back(cartesian_pos3D);

                            }

                            // TODO: optionals
                            //
                            // shapeReferencePoint    CartesianPosition3d OPTIONAL,
                            // height                 StandardLength12b OPTIONAL

                            break;

                        case Vanetza_ITS2_Shape_PR_elliptical:
                            perception_region.perception_region_shape.choice = Vanetza_ITS2_Shape_PR_elliptical;

                            perception_region.perception_region_shape.elliptical.semi_major_axis_length.value = asn1_perception_region->perceptionRegionShape.choice.elliptical.semiMajorAxisLength;
                            perception_region.perception_region_shape.elliptical.semi_minor_axis_length.value = asn1_perception_region->perceptionRegionShape.choice.elliptical.semiMinorAxisLength;

                            // TODO: optionals
                            //
                            // shapeReferencePoint    CartesianPosition3d OPTIONAL,
                            // orientation            Wgs84AngleValue OPTIONAL,
                            // height                 StandardLength12b OPTIONAL

                            break;

                        case Vanetza_ITS2_Shape_PR_radial:
                            perception_region.perception_region_shape.choice = Vanetza_ITS2_Shape_PR_radial;

                            perception_region.perception_region_shape.radial.range.value = asn1_perception_region->perceptionRegionShape.choice.radial.range;
                            perception_region.perception_region_shape.radial.stationary_horizontal_opening_angle_start.value = asn1_perception_region->perceptionRegionShape.choice.radial.horizontalOpeningAngleStart;
                            perception_region.perception_region_shape.radial.stationary_horizontal_opening_angle_end.value = asn1_perception_region->perceptionRegionShape.choice.radial.horizontalOpeningAngleEnd;

                            // TODO: optionals
                            //
                            // shapeReferencePoint                      CartesianPosition3d OPTIONAL,
                            // verticalOpeningAngleStart                CartesianAngleValue OPTIONAL,
                            // verticalOpeningAngleEnd                  CartesianAngleValue OPTIONAL


                            break;

                        case Vanetza_ITS2_Shape_PR_radialShapes:
                            perception_region.perception_region_shape.choice = Vanetza_ITS2_Shape_PR_radialShapes;

                            perception_region.perception_region_shape.radial_shapes.ref_point_id.value = asn1_perception_region->perceptionRegionShape.choice.radialShapes.refPointId;
                            perception_region.perception_region_shape.radial_shapes.x_coordinate.value = asn1_perception_region->perceptionRegionShape.choice.radialShapes.xCoordinate;
                            perception_region.perception_region_shape.radial_shapes.y_coordinate.value = asn1_perception_region->perceptionRegionShape.choice.radialShapes.yCoordinate;

                            // radialShapesList    RadialShapesList
                            for(int k = 0; k <  asn1_perception_region->perceptionRegionShape.choice.radialShapes.radialShapesList.list.count; k++)
                            {
                                const Vanetza_ITS2_RadialShapeDetails_t* asn1_radial_shape_details = asn1_perception_region->perceptionRegionShape.choice.radialShapes.radialShapesList.list.array[k];
                                etsi_its_cpm_ts_msgs::msg::RadialShapeDetails radial_shape_details;

                                radial_shape_details.range.value = asn1_radial_shape_details->range;
                                radial_shape_details.horizontal_opening_angle_start.value = asn1_radial_shape_details->horizontalOpeningAngleStart;
                                radial_shape_details.horizontal_opening_angle_end.value = asn1_radial_shape_details->horizontalOpeningAngleEnd;

                                // TODO: optionals
                                //
                                // verticalOpeningAngleStart      CartesianAngleValue OPTIONAL,
                                // verticalOpeningAngleEnd        CartesianAngleValue OPTIONAL

                                perception_region.perception_region_shape.radial_shapes.radial_shapes_list.array.push_back(radial_shape_details);

                            }

                            // TODO: optionals
                            //
                            // zCoordinate         CartesianCoordinateSmall OPTIONAL,
                            
                            break;
                        
                        default:
                            break;
                        }
                    }

                    perception_region.shadowing_applies = asn1_perception_region->shadowingApplies;

                    // TODO: optionals
                    //
                    // sensorIdList                 SequenceOfIdentifier1B OPTIONAL,
                    // numberOfPerceivedObjects     CardinalNumber1B OPTIONAL,
                    // perceivedObjectIds           PerceivedObjectIds OPTIONAL,

                    wrapped_cont.container_data_perception_region_container.array.push_back(perception_region);
                }
                
                break;

            // Perceived Object
            case 5:
                
                wrapped_cont.container_data_perceived_object_container.number_of_perceived_objects.value = asn1_wrapped_cont->containerData.choice.PerceivedObjectContainer.numberOfPerceivedObjects;

                for(int j = 0; j < asn1_wrapped_cont->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.count; j++)
                {  
                    const Vanetza_ITS2_PerceivedObject_t* asn1_perceived_object = asn1_wrapped_cont->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.array[j];
                    etsi_its_cpm_ts_msgs::msg::PerceivedObject perceived_object;

                    if(asn1_perceived_object->objectId)
                    {
                        perceived_object.object_id_is_present = true;
                        perceived_object.object_id.value = static_cast<short unsigned int>(*asn1_perceived_object->objectId);
                    }

                    perceived_object.measurement_delta_time.value = asn1_perceived_object->measurementDeltaTime;
                    perceived_object.position.x_coordinate.value.value = asn1_perceived_object->position.xCoordinate.value;
                    perceived_object.position.x_coordinate.confidence.value = asn1_perceived_object->position.xCoordinate.confidence;

                    perceived_object.position.y_coordinate.value.value = asn1_perceived_object->position.yCoordinate.value;
                    perceived_object.position.y_coordinate.confidence.value = asn1_perceived_object->position.yCoordinate.confidence;

                    if(asn1_perceived_object->position.zCoordinate){
                        perceived_object.position.z_coordinate_is_present = true;
                        perceived_object.position.z_coordinate.value.value = asn1_perceived_object->position.zCoordinate->value;
                        perceived_object.position.z_coordinate.confidence.value = asn1_perceived_object->position.zCoordinate->confidence;
                    }

                    // TODO: optionals
                    //
                    // velocity                                          Velocity3dWithConfidence OPTIONAL,
                    // acceleration                                      Acceleration3dWithConfidence OPTIONAL,
                    // angles                                            EulerAnglesWithConfidence OPTIONAL,
                    // zAngularVelocity                                  CartesianAngularVelocityComponent OPTIONAL,
                    // lowerTriangularCorrelationMatrices                LowerTriangularPositiveSemidefiniteMatrices OPTIONAL,
                    // objectDimensionZ                                  ObjectDimension OPTIONAL,
                    // objectDimensionY                                  ObjectDimension OPTIONAL,
                    // objectDimensionX                                  ObjectDimension OPTIONAL,
                    // objectAge                                         DeltaTimeMilliSecondSigned (0..2047) OPTIONAL,
                    // objectPerceptionQuality                           ObjectPerceptionQuality OPTIONAL,
                    // sensorIdList                                      SequenceOfIdentifier1B OPTIONAL,
                    // classification                                    ObjectClassDescription OPTIONAL,
                    // mapPosition                                       MapPosition OPTIONAL,

                    wrapped_cont.container_data_perceived_object_container.perceived_objects.array.push_back(perceived_object);
                }

                break;
            
            default:
                break;
            }
            
            msg->payload.cpm_containers.value.array.push_back(wrapped_cont);
        }

    }

    return msg;
}

/*
vanetza::asn1::Cam convertCam(ros_etsi_its_msgs::msg::CAM::ConstSharedPtr ptr)
{
    vanetza::asn1::Cam msg;

    ItsPduHeader_t& header = msg->header;
    header.protocolVersion = ptr->its_header.protocol_version;
    header.messageID = ptr->its_header.message_id;
    header.stationID = ptr->its_header.station_id;

    CoopAwareness_t& ca = msg->cam;
    ca.generationDeltaTime = ptr->generation_delta_time;

    BasicContainer_t& basic = ca.camParameters.basicContainer;
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

    HighFrequencyContainer_t& hfc = ca.camParameters.highFrequencyContainer;
    hfc.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;

    BasicVehicleContainerHighFrequency_t& bvc = hfc.choice.basicVehicleContainerHighFrequency;
    bvc.heading.headingValue = ptr->high_frequency_container.heading.value;
    bvc.heading.headingConfidence = ptr->high_frequency_container.heading.confidence;
    bvc.speed.speedValue = ptr->high_frequency_container.speed.value;
    bvc.speed.speedConfidence = ptr->high_frequency_container.speed.confidence;
    bvc.driveDirection = ptr->high_frequency_container.drive_direction.value;
    bvc.longitudinalAcceleration.longitudinalAccelerationValue =
        ptr->high_frequency_container.longitudinal_acceleration.value;
    bvc.longitudinalAcceleration.longitudinalAccelerationConfidence =
        ptr->high_frequency_container.longitudinal_acceleration.confidence;
    bvc.curvature.curvatureValue = ptr->high_frequency_container.curvature.value;
    bvc.curvature.curvatureConfidence = ptr->high_frequency_container.curvature.confidence;
    bvc.curvatureCalculationMode = ptr->high_frequency_container.curvature_calculation_mode.value;
    bvc.yawRate.yawRateValue = ptr->high_frequency_container.yaw_rate.value;
    bvc.yawRate.yawRateConfidence = ptr->high_frequency_container.yaw_rate.confidence;
    bvc.vehicleLength.vehicleLengthValue = ptr->high_frequency_container.vehicle_length.value;
    bvc.vehicleLength.vehicleLengthConfidenceIndication =
        ptr->high_frequency_container.vehicle_length.confidence_indication;
    bvc.vehicleWidth = ptr->high_frequency_container.vehicle_width.value;
    if (ptr->high_frequency_container.has_acceleration_control)
    {
        bvc.accelerationControl = vanetza::asn1::allocate<AccelerationControl_t>();
        bvc.accelerationControl->buf = static_cast<uint8_t*>(vanetza::asn1::allocate(1));
        bvc.accelerationControl->size = 1;
        bvc.accelerationControl->buf[0] = reverse_byte(ptr->high_frequency_container.acceleration_control.value);
    }

    if (ptr->has_low_frequency_container)
    {
        ca.camParameters.lowFrequencyContainer = vanetza::asn1::allocate<LowFrequencyContainer_t>();
        ca.camParameters.lowFrequencyContainer->present = LowFrequencyContainer_PR_basicVehicleContainerLowFrequency;

        BasicVehicleContainerLowFrequency& bvcl = ca.camParameters.lowFrequencyContainer->choice.basicVehicleContainerLowFrequency;
        bvcl.vehicleRole = ptr->low_frequency_container.vehicle_role.value;
        bvcl.exteriorLights.buf = static_cast<uint8_t*>(vanetza::asn1::allocate(1));
        bvcl.exteriorLights.size = 1;
        bvcl.exteriorLights.buf[0] = reverse_byte(ptr->low_frequency_container.exterior_lights.value);

        for (const ros_etsi_its_msgs::msg::PathPoint& path_point : ptr->low_frequency_container.path_history.points)
        {
            PathPoint_t* pathPoint = vanetza::asn1::allocate<PathPoint_t>();
            if (path_point.path_delta_time.value != ros_etsi_its_msgs::msg::PathDeltaTime::UNAVAILABLE)
            {
                pathPoint->pathDeltaTime = vanetza::asn1::allocate<PathDeltaTime_t>();
                *pathPoint->pathDeltaTime = path_point.path_delta_time.value;
            }
            pathPoint->pathPosition.deltaAltitude = path_point.path_position.delta_altitude;
            pathPoint->pathPosition.deltaLatitude = path_point.path_position.delta_latitude;
            pathPoint->pathPosition.deltaLongitude = path_point.path_position.delta_longitude;
            ASN_SEQUENCE_ADD(&bvcl.pathHistory, pathPoint);
        }
    }

    return msg;
}*/

} // namespace v2x_stack_btp

