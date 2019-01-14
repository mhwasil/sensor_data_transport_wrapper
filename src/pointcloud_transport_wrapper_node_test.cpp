#include <realsense_camera_wrapper/realsense_camera.h>
// include Santosh package https://github.com/sthoduka/zmq_pointcloud_transport/
#include <camera_ros_wrapper/zmq_pointcloud_transport.h>
#include <librealsense2/rs.hpp>
#include <ros/ros.h>
#include <iostream>

#include <boost/thread/thread.hpp>
#include <boost/chrono/include.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include "pcl_ros/point_cloud.h"
#include <sensor_msgs/PointCloud2.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

int main(int argc, char *argv[]) 
{
    ros::init(argc, argv, "realsense2_camera_node");
    ros::NodeHandle nh;
    ros::Publisher pub_pcl;
    uint16_t width = 640;
    uint16_t height = 480;
    RealsenseCamera device(width, height);
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    rs2::frame color;
    rs2::frame depth; 
    double t_start;
    double t_end;

    pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("cloud_output", 1);

	std::string base_link = "realsense2_base_link";
    std::string depth_frame_id = "realsense2_depth_frame";
    std::string depth_optical_frame_id = "realsense2_depth_optical_frame";
    std::string rgb_frame_id = "realsense2_rgb_frame";
    std::string rgb_optical_frame_id = "realsense2_rgb_optical_frame_id";

    while(nh.ok())
    {
        std::chrono::milliseconds::rep time_stamp;
        t_start = ros::Time::now().toSec();   
        auto cloud = device.get_pointcloud(time_stamp);
        t_end = ros::Time::now().toSec();   
        ROS_WARN_STREAM("time required for calculating point cloud " << (t_end - t_start));
        // Publish pcl
        sensor_msgs::PointCloud2 ros_cloud;
        pcl::PCLPointCloud2::Ptr pc2(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(*cloud, *pc2);
        pcl_conversions::fromPCL(*pc2, ros_cloud);
        ros_cloud.header.stamp = ros::Time::now();
        ros_cloud.header.frame_id = depth_optical_frame_id; 

        ZPT::ZMQPointCloudTransport zpt;
        zpt.init(ZPT::SENDER);
        usleep(100000);
        zpt.publish(pc2);
        
        //pub_pcl.publish(ros_cloud);

        ros::Rate r(30);
        r.sleep();
    }
}
