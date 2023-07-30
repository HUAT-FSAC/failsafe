#ifndef MAIN_H_
#define MAIN_H_

#include "ros/ros.h"

// msg include
#include "common_msgs/HUAT_ASENSING.h"
#include "sensor_msgs/PointCloud2.h"


ros::NodeHandle nh;

ros::Subscriber cam_sub;
ros::Subscriber lidar_sub;
ros::Subscriber imu_sub;
ros::Publisher control_pub;

// function define
void init_subscriber();
void lidar_callback(const sensor_msgs::PointCloud2ConstPtr &original_cloud_ptr);
void imu_callback(const common_msgs::HUAT_ASENSING::ConstPtr &msgs);

#endif