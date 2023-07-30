#include "main.hpp"

void init_subscriber()
{
    // cam_sub = nh.subscribe("/pylon_camera_node/image_raw", 1, cam_callback);
    lidar_sub = nh.subscribe("/velodyne_points", 2, lidar_callback);
    imu_sub = nh.subscribe("/INS/ASENSING", 1, imu_callback);
}

// sensor_msgs/Image
void lidar_callback(const sensor_msgs::PointCloud2ConstPtr &original_cloud_ptr)
{

}

void imu_callback(const common_msgs::HUAT_ASENSING::ConstPtr &msgs)
{
    // if the msg is corrupted

}

// void cam_callback()
// {

// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "failsafe");

    ros::spin();
}