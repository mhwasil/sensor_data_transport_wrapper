#include <camera_ros_wrapper/pointcloud_transport_wrapper.h>
#include <realsense_camera_wrapper/realsense_camera_device_wrapper.h>
// include Santosh package https://github.com/sthoduka/zmq_pointcloud_transport/
#include <camera_ros_wrapper/zmq_pointcloud_transport.h>
#include <librealsense2/rs.hpp>
#include <ros/ros.h>
#include <iostream>
#include <chrono>

#include <boost/thread/thread.hpp>
#include <boost/chrono/include.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

//

PointCloudTransportWrapper::PointCloudTransportWrapper( boost::shared_ptr<RealsenseCamera> realsense_interface):
realsense_interface_(realsense_interface)
{
    pub_pcl_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_output", 1);

	// std::string base_link_ = "realsense2_base_link";
    // std::string depth_frame_id_ = "realsense2_depth_frame";
    // std::string depth_optical_frame_id_ = "realsense2_depth_optical_frame";
    // std::string rgb_frame_id_ = "realsense2_rgb_frame";
    // std::string rgb_optical_frame_id_ = "realsense2_rgb_optical_frame_id";
    
    //device_ = RealsenseCamera::Ptr(new RealsenseCamera(480,640));
    std::string frame_id = "frame_id";
    bool transform = false;
    get_zmq_pointcloud(transform, frame_id);
    
}

PointCloudTransportWrapper::~PointCloudTransportWrapper()
{ 
}

void PointCloudTransportWrapper::get_zmq_pointcloud(bool transform, std::string target_frame)
{
    //PointCloudTransportWrapper::get_pointcloud();
    realsense_interface_->get_pointcloud(cloud_);
    if (transform)
    {
        std::cout<<"None";
    }
    else
    {
        pcl::PCLPointCloud2::Ptr pc2(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(*cloud_, *pc2);
        auto stamp = std::chrono::system_clock::now();
        std::string frame_id = "realsense2_depth_frame";
        
        pcl::PCLHeader header;
        //header.stamp = stamp;
        header.frame_id = frame_id;

        pc2->header = header;
        
        ZPT::ZMQPointCloudTransport zpt;
        zpt.init(ZPT::SENDER);
        usleep(100000);

        zpt.publish(pc2);
    }

}

int main(int argc, char *argv[]) 
{
    ros::init(argc, argv, "pointcloud_transport_wrapper");
    
    std::string frame_id = "frame_id";
    bool transform = false;

    boost::shared_ptr<RealsenseCamera> realsense_interface = boost::make_shared<RealsenseCamera>(480,640);
    PointCloudTransportWrapper pcl_wrapper(realsense_interface);
    //pcl_wrapper.get_zmq_pointcloud(transform, frame_id);
    ros::spin();
}
