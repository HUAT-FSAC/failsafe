#ifndef MAIN_H_
#define MAIN_H_

#include "ros/ros.h"

// spec msg include
#include "common_msgs/HUAT_ASENSING.h"
#include "common_msgs/vehicle_cmd.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"

#include <fcntl.h> // Contains file controls like O_RDWR
#include <fstream>
// control msg

// const defination
enum FailureType {
    FAILURE_NO = 0,
    FAILURE_CAM = 1,
    FAILURE_IMU = 2,
    FAILURE_LIDAR = 3
};

// function define
void init(ros::NodeHandle nh);

void cam_callback(const sensor_msgs::Image::ConstPtr &msg);
void lidar_callback(const sensor_msgs::PointCloud2ConstPtr &original_cloud_ptr);
void imu_callback(const common_msgs::HUAT_ASENSING::ConstPtr &msgs);

void alert(int type);
void checkOnce();
void checkRuntime();
void hardwareCheck();

// varibles
ros::Subscriber cam_sub;
ros::Subscriber lidar_sub;
ros::Subscriber imu_sub;
ros::Publisher control_pub;

sensor_msgs::PointCloud2 _cloud;
common_msgs::HUAT_ASENSING _pos;
sensor_msgs::Image _image;

common_msgs::vehicle_cmd cmd;

std::ifstream file;
std::string buffer;
std::string cam_eth;
std::string lidar_eth;
bool keep_running;

bool runtime_init_flag = false;
bool _ok = true;
#endif