Sensor data transport via ros communication (topic and service) and zero mq wrapper.
This package is a wrapper for transporting sensor data such as pointcloud, images (color and depth).
This package implements [realsense_camera_wrapper](https://github.com/abhishek098/realsense_camera_wrapper) which is independent of ros for getting sensor data.
The zmq communication is based on [zmq_pointcloud_transport](https://github.com/sthoduka/zmq_pointcloud_transport).
The sensor data can be transported with different methods:<br>
* ROS
** Topic
** Service
* zmq
** Publisher-subscriber
** Server-client
