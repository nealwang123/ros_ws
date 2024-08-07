#include "rclcpp/rclcpp.hpp"

#include <ars548_interface/msg/objects.hpp>
#include <ars548_interface/msg/object_list.hpp>
#include <ars548_interface/msg/detections.hpp>
#include <ars548_interface/msg/detection_list.hpp>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <sensor_msgs/PointCloud.h>
// #include <geometry_msgs/Point32.h>
// #include <sensor_msgs/ChannelFloat32.h>
#include "sensor_msgs/msg/point_cloud2.hpp"  
#include "std_msgs/msg/header.hpp"  
#include <pcl/conversions.h>  // 确保包含此头文件  
#include <pcl/point_cloud.h>  
#include <pcl/point_types.h>  
#include <pcl_conversions/pcl_conversions.h>  
// ros::Publisher objects_marker_pub;
// ros::Publisher detections_cloud_pub;
rclcpp::Subscription<ars548_interface::msg::DetectionList>::SharedPtr det_list_sub ;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr detections_cloud_pub ;
rclcpp::Logger global_logger = rclcpp::get_logger("global_logger");
// void objectReceive(const ars548_msg::ObjectList& msg)
// {
//     uint size = msg.object_array.size();
//     visualization_msgs::Marker my_marker; 
//     visualization_msgs::MarkerArray marker_array;
//     marker_array.markers.clear();

//     if(size>0)
//     {
//         for(uint i=0; i<size; i++)
//         {
//             my_marker.header.frame_id = "/world";
//             my_marker.header.stamp = msg.object_array[i].header.stamp;
//             my_marker.ns = "object_shapes";
            
//             my_marker.id = msg.object_array[i].u_ID; 
//             my_marker.type = visualization_msgs::Marker::CUBE;
//             my_marker.action = visualization_msgs::Marker::ADD; 
//             my_marker.pose.position.x = msg.object_array[i].u_Position_X;
//             my_marker.pose.position.y = msg.object_array[i].u_Position_Y;
//             my_marker.pose.position.z = msg.object_array[i].u_Position_Z;

//             my_marker.pose.orientation.x = 0.0;
//             my_marker.pose.orientation.y = 0.0;
//             my_marker.pose.orientation.z = sin(msg.object_array[i].u_Position_Orientation/2);
//             my_marker.pose.orientation.w = cos(msg.object_array[i].u_Position_Orientation/2);

//             if((msg.object_array[i].u_Shape_Length_Edge_Mean>0.2)||(msg.object_array[i].u_Shape_Width_Edge_Mean>0.2))
//             {
//                 my_marker.scale.x = msg.object_array[i].u_Shape_Length_Edge_Mean;
//                 my_marker.scale.y = msg.object_array[i].u_Shape_Width_Edge_Mean;
//                 my_marker.scale.z = (msg.object_array[i].u_Shape_Length_Edge_Mean+msg.object_array[i].u_Shape_Width_Edge_Mean)/2;
//             }
//             else
//             {
//                 my_marker.scale.x = 0.2;
//                 my_marker.scale.y = 0.2;
//                 my_marker.scale.z = 0.2;
//             }

//             my_marker.color.r = 0.0f;
//             my_marker.color.g = 1.0f;
//             my_marker.color.b = 0.0f;
//             my_marker.color.a = 1.0;

//             my_marker.lifetime = ros::Duration(0.5);

//             marker_array.markers.push_back(my_marker);
//         }

//         objects_marker_pub.publish(marker_array);
//     }
// }

void detectionReceive(const ars548_interface::msg::DetectionList& msg)
{
    uint size = msg.detection_array.size();
    // RCLCPP_INFO(global_logger, "detectionReceive PointCloud with %lu points", size); 
    // 创建点云数据，可以根据需要自定义点云  
    pcl::PointCloud<pcl::PointXYZ> cloud;  
    cloud.width = size;  
    cloud.height = 1;  
    cloud.is_dense = false;  
    cloud.points.resize(cloud.width * cloud.height);
    if(size>0)
    {
        cloud.header.frame_id = "/world";
        // cloud.header.stamp = msg.detection_array[0].header.stamp;
        cloud.points.resize(size); // Resize to the actual number of points
        for(uint i=0;i<size;i++) 
        {
            cloud.points[i].x = msg.detection_array[i].f_x;
            cloud.points[i].y = msg.detection_array[i].f_y; 
            cloud.points[i].z = msg.detection_array[i].f_z; 
        }
    }
    
    // 转换为 PointCloud2 消息  
    sensor_msgs::msg::PointCloud2 msg2;  
    pcl::toROSMsg(cloud, msg2);  
    msg2.header.frame_id = "map";  // 设置坐标系  
    // msg2.header.stamp = now();  
    detections_cloud_pub->publish(msg2);  
    RCLCPP_INFO(global_logger, "Publishing PointCloud with %lu points", cloud.points.size()); 
    

}

int main(int argc, char **argv)
{
    // ros::init(argc, argv, "info_convert_node");
    // ros::NodeHandle nh;
    // ros::NodeHandle private_nh("~");

    // ros::Subscriber obj_list_sub = nh.subscribe("/ars548_process/object_list", 10, &objectReceive);
    // ros::Subscriber det_list_sub = nh.subscribe("/ars548_process/detection_list", 10, &detectionReceive);
    // objects_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/ars548_process/object_marker", 10);
    // detections_cloud_pub = nh.advertise<sensor_msgs::PointCloud>("/ars548_process/detection_point_cloud", 10);

    // ros::spin();
    // ROS2 C++接口初始化
    rclcpp::init(argc, argv);
    // 创建ROS2节点对象并进行初始化                                                                   
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("info_convert_node");  
    det_list_sub = node->create_subscription<ars548_interface::msg::DetectionList>("/ars548_process/detection_list", 100, detectionReceive);
    detections_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/ars548_process/detection_point_cloud", 100); 
    // ros::spin();
    // 循环等待ROS2退出
    rclcpp::spin(node);    
    // 关闭ROS2 C++接口                                                                     
    rclcpp::shutdown();  
}