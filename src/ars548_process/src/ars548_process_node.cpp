#include <ars548_interface/msg/objects.hpp>
#include <ars548_interface/msg/object_list.hpp>
#include <ars548_interface/msg/detections.hpp>
#include <ars548_interface/msg/detection_list.hpp>
#include <ars548_interface/msg/radar_basic_status.hpp>
#include <ars548_interface/msg/acceleration_lateral_cog.hpp>
#include <ars548_interface/msg/acceleration_longitudinal_cog.hpp>
#include <ars548_interface/msg/characteristic_speed.hpp>
#include <ars548_interface/msg/driving_direction.hpp>
#include <ars548_interface/msg/steering_angle_front_axle.hpp>
#include <ars548_interface/msg/velocity_vehicle.hpp>
#include <ars548_interface/msg/yaw_rate.hpp>
#include "rclcpp/rclcpp.hpp"
#include <thread>

#include "udp_interface.h"
#include "data_struct.h"
#include "data_process.h"
#include "converttype.h"

UdpInterface udp_io;
DataProcess data_process;
ConvertType cvt;

char rec_data[40000];
char send_data[1024];
int rec_length;
struct sockaddr_in source_address;

// RadarObjectList object_list;
RadarDetectionList detection_list;
// RadarBasicStatus basic_status;


// ros::Publisher objects_pub;
// rclcpp::Node detections_pub;
// rclcpp::Node status_pub;
rclcpp::Publisher<ars548_interface::msg::DetectionList>::SharedPtr detections_pub ;
rclcpp::Logger global_logger = rclcpp::get_logger("global_logger");
void publishDetectionList(RadarDetectionList list)
{
    ars548_interface::msg::Detections det;
    ars548_interface::msg::DetectionList det_list;
    int i;
    det_list.detection_array.clear();

    det_list.service_id = list.serviceID;
    det_list.method_id = list.MethodID;
    det_list.data_length = list.data_length;
    det_list.client_id = list.clientID;
    det_list.session_id = list.sessionID;
    det_list.protocol_version = list.protocol_version;
    det_list.interface_version = list.interface_version;
    det_list.message_type = list.message_type;
    det_list.return_code = list.return_code;
    det_list.crc = list.CRC;
    det_list.length = list.Length;
    det_list.sqc = list.SQC;
    det_list.data_id = list.DataID;
    det_list.timestamp_nanoseconds = list.Timestamp_Nanoseconds;
    det_list.timestamp_seconds = list.Timestamp_Seconds;
    det_list.timestamp_sync_status = list.Timestamp_SyncStatus;
    det_list.event_data_qualifier = list.EventDataQualifier;
    det_list.extended_qualifier = list.ExtendedQualifier;
    det_list.origin_invalid_flags = list.Origin_InvalidFlags;
    det_list.origin_xpos = list.Origin_Xpos;
    det_list.origin_xstd = list.Origin_Xstd;
    det_list.origin_ypos = list.Origin_Ypos;
    det_list.origin_ystd = list.Origin_Ystd;
    det_list.origin_zpos = list.Origin_Zpos;
    det_list.origin_zstd = list.Origin_Zstd;
    det_list.origin_roll = list.Origin_Roll;
    det_list.origin_rollstd = list.Origin_Rollstd;
    det_list.origin_pitch = list.Origin_Pitch;
    det_list.origin_pitchstd = list.Origin_Pitchstd;
    det_list.origin_yaw = list.Origin_Yaw;
    det_list.origin_yawstd = list.Origin_Yawstd;
    det_list.list_invalid_flags = list.List_InvalidFlags;
    det_list.list_rad_vel_domain_min = list.List_RadVelDomain_Min;
    det_list.list_rad_vel_domain_max = list.List_RadVelDomain_Max;
    det_list.list_num_of_detections = list.List_NumOfDetections;
    det_list.aln_azimuth_correction = list.Aln_AzimuthCorrection;
    det_list.aln_elevation_correction = list.Aln_ElevationCorrection;
    det_list.aln_status = list.Aln_Status;

    for(i=0;i<list.List_NumOfDetections;i++)
    {
        det.header.frame_id = "/world";
        det.header.stamp = rclcpp::Clock().now();

        det.f_x = list.detection_array[i].f_x;
        det.f_y = list.detection_array[i].f_y;
        det.f_z = list.detection_array[i].f_z;
        det.u_invalid_flags = list.detection_array[i].u_InvalidFlags;
        det.f_range_rate = list.detection_array[i].f_RangeRate;
        det.f_range_rate_std = list.detection_array[i].f_RangeRateSTD;
        det.s_rcs = list.detection_array[i].s_RCS;
        det.u_measurement_id = list.detection_array[i].u_MeasurementID;
        det.u_positive_predictive_value = list.detection_array[i].u_PositivePredictiveValue;
        det.u_classification = list.detection_array[i].u_Classification;
        det.u_multi_target_probability = list.detection_array[i].u_MultiTargetProbability;
        det.u_object_id = list.detection_array[i].u_ObjectID;
        det.u_ambiguity_flag = list.detection_array[i].u_AmbiguityFlag;
        det.u_sort_index = list.detection_array[i].u_SortIndex;

        det_list.detection_array.push_back(det);
    }

    detections_pub->publish(det_list);  
    RCLCPP_INFO(global_logger, "Published: '%d'", det_list.timestamp_seconds);  // 打印日志  
}

void ProcessRadarData(char *data, int len)
{
    switch(len)
    {
        // case 9401:
        //     if(data_process.processObjectListMessage(data,&object_list))
        //     {
        //         if(object_list.ObjectList_NumOfObjects>0)
        //             publishObjectList(object_list);    
        //     }
        // break;

        case 35336:
            if(data_process.processDetectionListMessage(data,&detection_list))
            {
                if(detection_list.List_NumOfDetections)
                    publishDetectionList(detection_list);
            }
        break;

        // case 84:
        //     if(data_process.processBasicStatusMessage(data,&basic_status))
        //     {
        //         publishBasicStatus(basic_status);
        //     }
        // break;

        default:
        break;
    }
}

//thread for receive radar data
void receiveThread()
{
    cout<< "receiveThread()"<<endl;
    // while(ros::ok())
    while(rclcpp::ok())
    {
        // cout<< "rclcpp::ok()"<<endl;
        udp_io.receiveFromRadar((struct sockaddr *)&source_address,rec_data,rec_length);
        //打印雷达地址，用于连接多个雷达时区分哪一个雷达
        // cout<< source_address.sin_addr.s_addr<<endl;
        if(rec_length>0)
        {
            ProcessRadarData(rec_data,rec_length);    
        }
    }

}


int main(int argc, char **argv)
{
    //ros::init(argc, argv, "ars548_process_node");
    // ros::NodeHandle nh;
    // ros::NodeHandle private_nh("~");
    // objects_pub = nh.advertise<ars548_msg::ObjectList>("/ars548_process/object_list", 10);
    // detections_pub = nh.advertise<ars548_msg::DetectionList>("/ars548_process/detection_list", 10);
    // status_pub = nh.advertise<ars548_msg::RadarBasicStatus>("/ars548_process/radar_status", 10);

    // ros::Subscriber acc_lateral_sub = nh.subscribe("/ars548_process/acc_lateral_cog", 10, &AccLateralCogCallBack);
    // ros::Subscriber acc_longitudinal_sub = nh.subscribe("/ars548_process/acc_longitudinal_cog", 10, &AccLongitudinalCogCallBack);
    // ros::Subscriber characteristic_speed_sub = nh.subscribe("/ars548_process/characteristic_speed", 10, &CharacteristicSpeedCallBack);
    // ros::Subscriber driving_direction_sub = nh.subscribe("/ars548_process/driving_direction", 10, &DrivingDirectionCallBack);
    // ros::Subscriber steering_angle_sub = nh.subscribe("/ars548_process/steering_angle", 10, &SteeringAngleCallBack);
    // ros::Subscriber velocity_vehicle_sub = nh.subscribe("/ars548_process/velocity_vehicle", 10, &VelocityVehicleCallBack);
    // ros::Subscriber yaw_rate_sub = nh.subscribe("/ars548_process/yaw_rate", 10, &YawRateCallBack);

    // ROS2 C++接口初始化
    rclcpp::init(argc, argv);
    // 创建ROS2节点对象并进行初始化                                                                   
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ars548_process_node");  
    detections_pub = node->create_publisher<ars548_interface::msg::DetectionList>("/ars548_process/detection_list", 100); 
    

    int ret=udp_io.initUdpMulticastServer("224.0.2.2",42102);
    if(ret<0)
    {
        cout<<"initUdpMulticastServer error!"<<endl;
        return 0;
    }
    cout<<"initUdpMulticastServer ok!"<<ret<<endl;
    // udp_io.initUdpUnicastClient("10.13.1.113",42101,42401);

    std::thread rec_thread = std::thread(receiveThread);  

    // ros::spin();
    // 循环等待ROS2退出
    rclcpp::spin(node);    
    // 关闭ROS2 C++接口                                                                     
    rclcpp::shutdown();  

    return 0;
}


