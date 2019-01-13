/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil, Santosh Thoduka
 *
 */
#ifndef POINTCLOUD_TRANSPORT_WRAPPER_H
#define POINTCLOUD_TRANSPORT_WRAPPER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include "pcl_ros/point_cloud.h"
#include <sensor_msgs/PointCloud2.h>

#include <realsense_camera_wrapper/realsense_camera.h>

class PointCloudTransportWrapper
{
    public:
        PointCloudTransportWrapper(boost::shared_ptr<RealsenseCamera> realsense_interface = nullptr);
        virtual ~PointCloudTransportWrapper();
        

    private:
        void get_pointcloud();
        ros::NodeHandle nh_;
        //RealsenseCamera::Ptr device_;
        boost::shared_ptr<RealsenseCamera> realsense_interface_;

    public:
        void get_ros_pointcloud(bool transform, std::string target_frame);
        void get_zmq_pointcloud(bool transform, std::string target_frame);
        void get_ros_image();
        void get_zmq_image();
    
    private:
        //ptr_cloud cloud_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
        ros::Publisher pub_pcl_;

};

#endif  // POINTCLOUD_TRANSPORT_WRAPPER_H
