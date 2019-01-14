#include <sensor_data_transport_wrapper/pointcloud_transport_wrapper.h>
#include <realsense_camera_wrapper/realsense_camera.h>
// include Santosh package https://github.com/sthoduka/zmq_pointcloud_transport/
#include <sensor_data_transport_wrapper/zmq_pointcloud_transport.h>
#include <librealsense2/rs.hpp>
#include <ros/ros.h>
#include <iostream>
#include <chrono>

#include <boost/thread/thread.hpp>
#include <boost/chrono/include.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

PointCloudTransportWrapper::PointCloudTransportWrapper( ros::NodeHandle nh, boost::shared_ptr<RealsenseCamera>realsense_interface):
realsense_interface_(realsense_interface),
nh_(nh)
{
    pub_pcl_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_output", 1);

	// std::string base_link_ = "realsense2_base_link";
    // std::string depth_frame_id_ = "realsense2_depth_frame";
    // std::string depth_optical_frame_id_ = "realsense2_depth_optical_frame";
    // std::string rgb_frame_id_ = "realsense2_rgb_frame";
    // std::string rgb_optical_frame_id_ = "realsense2_rgb_optical_frame_id";
    //uint16_t width = 640;
    //uint16_t height = 480;
    //realsense_interface_ = boost::make_shared<RealsenseCamera>(width,height);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //cloud_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >();
    //cloud_ = boost::make_shared <pcl::PointCloud<pcl::PointXYZRGB> > (new pcl::PointCloud<pcl::PointXYZRGB>);
    //cloud_ = cloud_.makeShared();
    
}

PointCloudTransportWrapper::~PointCloudTransportWrapper()
{ 
}

void PointCloudTransportWrapper::get_zmq_pointcloud(bool transform, std::string target_frame)
{
    std::chrono::milliseconds::rep time_stamp;
    auto cloud_new = realsense_interface_->get_pointcloud(time_stamp);
    std::cout<<cloud_new->header.frame_id;
    if (transform)
    {
        std::cout<<"None";
    }
    else
    {
        sensor_msgs::PointCloud2 ros_cloud;
        pcl::PCLPointCloud2::Ptr pc2(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(*cloud_new, *pc2);
        auto stamp = std::chrono::system_clock::now();
        std::string frame_id = "realsense2_depth_frame";
        pcl_conversions::fromPCL(*pc2, ros_cloud);
        ros_cloud.header.stamp = ros::Time::now();
        ros_cloud.header.frame_id = frame_id; 
        
        pcl::PCLHeader header;
        //header.stamp = stamp;
        header.frame_id = frame_id;

        pc2->header = header;
        
        ZPT::ZMQPointCloudTransport zpt;
        zpt.init(ZPT::SENDER);
        usleep(100000);

        zpt.publish(pc2);
        pub_pcl_.publish(ros_cloud);
    }

}

int main(int argc, char *argv[]) 
{
    ros::init(argc, argv, "pointcloud_transport_wrapper");
    
    std::string frame_id = "frame_id";
    bool transform = false;
    uint16_t width = 640;
    uint16_t height = 480;
    ros::NodeHandle nh;

    boost::shared_ptr<RealsenseCamera> realsense_interface_ = boost::make_shared<RealsenseCamera>(width,height);
    
    while(nh.ok())
    {
        PointCloudTransportWrapper pcl_wrapper(nh, realsense_interface_);
        //pub_pcl.publish(ros_cloud);
        std::string frame_id = "frame_id";
        bool transform = false;
        pcl_wrapper.get_zmq_pointcloud(transform, frame_id);
    

        ros::Rate r(30);
        r.sleep();
    }
    //ros::spin();
}
