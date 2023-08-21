#ifndef MAIN_H_
#define MAIN_H_

#include "ros/ros.h"

// spec msg include
#include "common_msgs/HUAT_ASENSING.h"
#include "common_msgs/vehicle_cmd.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
// control msg

// const defination
enum FailureType {
  FAILURE_NO = -1,
  FAILURE_CAM = 0,
  FAILURE_IMU = 1,
  FAILURE_LIDAR = 2
};

// function define
void init(ros::NodeHandle nh);
void cam_callback(const sensor_msgs::Image::ConstPtr &msg);
void lidar_callback(const sensor_msgs::PointCloud2ConstPtr &original_cloud_ptr);
void imu_callback(const common_msgs::HUAT_ASENSING::ConstPtr &msgs);
void alert(int type);
bool checkOnce();

// varibles
ros::Subscriber cam_sub;
ros::Subscriber lidar_sub;
ros::Subscriber imu_sub;
ros::Publisher control_pub;

bool keep_running = false;

sensor_msgs::PointCloud2 _cloud;
common_msgs::HUAT_ASENSING _pos;
sensor_msgs::Image _image;
#endif