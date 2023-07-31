#ifndef MAIN_H_
#define MAIN_H_

#include "ros/ros.h"

// msg include
#include "common_msgs/HUAT_ASENSING.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "common_msgs/vehicle_cmd.h"
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
void lidar_callback(const sensor_msgs::PointCloud2ConstPtr &original_cloud_ptr);
void imu_callback(const common_msgs::HUAT_ASENSING::ConstPtr &msgs);
void alert(int type);

#endif